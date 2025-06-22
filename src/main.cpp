#include <SFML/Graphics.hpp>
#include "polyphysics.h"
#include <cmath>
#include <iostream>
#include <algorithm>  // for std::reverse

int main()
{
    const unsigned W = 800, H = 600;
    sf::RenderWindow window({W, H}, "Circle vs Triangle");
    window.setFramerateLimit(60);

    PhysicsEngine engine;
    engine.simulationPhysicsSettings.gravity                   = {0, 0};
    engine.simulationPhysicsSettings.restingVelocity           = 0.0f;
    engine.simulationPhysicsSettings.restingAngularVelocity    = 0.0f;
    // ← DESACTIVO TODO EL DRAG GLOBALMENTE
    engine.simulationPhysicsSettings.airDragCoefficient        = 0.0f;
    engine.simulationPhysicsSettings.rotationalDragCoefficient = 0.0f;

    // ——— 1) Círculo ———
    float r = 30.f;
    ph2dBodyId circleId = engine.addBody({100.f, 100.f}, createCircleCollider(r));
    engine.bodies[circleId].motionState.velocity = { 150.f,  80.f };

    // ——— 2) Triángulo equilátero ———
    float side   = 80.f;
    float height = side * std::sqrt(3.f) / 2.f;
    Vec2 triVerts[3] = {
        {   0.f,      -2*height/3 },  // vértice superior
        { -side/2,     height/3   },  // vértice izquierdo
        {  side/2,     height/3   }   // vértice derecho
    };

    // Si lo quieres en CW para que rote correctamente:
    std::reverse(triVerts, triVerts + 3);

    // Calculamos la posición inicial para que el triángulo
    // toque la esquina inferior derecha de la ventana:
    float startX = W - side * 0.5f;    // centro desplazado half-base hacia la izquierda
    float startY = H - height * (1.f/3.f); // centro un tercio de la altura hacia arriba

    ph2dBodyId triId = engine.addBody(
        { startX, startY },
        createConvexPolygonCollider(triVerts, 3)
    );
    engine.bodies[triId].motionState.velocity        = {-120.f, -100.f};
    engine.bodies[triId].motionState.angularVelocity =   1.2f;  // para que rote

    sf::Clock clock;
    while (window.isOpen())
    {
        // ——— Eventos ———
        sf::Event ev;
        while (window.pollEvent(ev))
            if (ev.type == sf::Event::Closed)
                window.close();

        // ——— Simulación ———
        float dt = clock.restart().asSeconds();
        engine.runSimulation(dt);

        auto& T = engine.bodies[triId].motionState;
        std::cout
            << "tri pos = " << T.pos.x << ", " << T.pos.y
            << "   vel = " << T.velocity.x << ", " << T.velocity.y
            << "\n";

        // ——— Rebote contra los bordes ———
        auto bounce = [&](ph2dBodyId id) {
            auto& B = engine.bodies[id];
            Vec2&  p = B.motionState.pos;
            Vec2&  v = B.motionState.velocity;

            // AABB sin rotar (aprox. suficiente para rebote)
            AABB a = B.getAABB();
            Vec2 mn = a.min(), mx = a.max();

            if (mn.x < 0.f) { p.x += -mn.x; v.x =  std::abs(v.x); }
            if (mx.x > W)  { p.x +=  W - mx.x; v.x = -std::abs(v.x); }
            if (mn.y < 0.f) { p.y += -mn.y; v.y =  std::abs(v.y); }
            if (mx.y > H)  { p.y +=  H - mx.y; v.y = -std::abs(v.y); }
        };
        bounce(circleId);
        bounce(triId);

        // ——— Render ———
        window.clear(sf::Color::Black);

        // Dibuja el círculo
        {
            auto& B = engine.bodies[circleId];
            sf::CircleShape shape(r);
            shape.setOrigin(r, r);
            shape.setPosition(B.motionState.pos.x, B.motionState.pos.y);
            shape.setFillColor(sf::Color::Green);
            window.draw(shape);
        }

        // Dibuja el triángulo
        {
            auto& B = engine.bodies[triId];
            sf::ConvexShape tri;
            tri.setPointCount(3);
            for (int i = 0; i < 3; ++i)
                tri.setPoint(i, sf::Vector2f(triVerts[i].x, triVerts[i].y));
            // Como origin está en (0,0) = centroide, rota alrededor de su centro
            tri.setOrigin(0.f, 0.f);
            tri.setPosition(B.motionState.pos.x, B.motionState.pos.y);
            tri.setRotation(B.motionState.rotation * 180.f / 3.14159265f);
            tri.setFillColor(sf::Color::Magenta);
            window.draw(tri);
        }

        window.display();
    }

    return 0;
}
