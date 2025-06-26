#include <SFML/Graphics.hpp>
#include "polyphysics.h" // Asumimos que todo tu código está en este header.
#include <iostream>

// Función para convertir de Vec2 de tu motor a sf::Vector2f de SFML
sf::Vector2f toSfmlVec(const Vec2& v) {
    return sf::Vector2f(v.x, v.y);
}

// Función para generar un color aleatorio para cada objeto
sf::Color getRandomColor() {
    return sf::Color(100 + rand() % 156, 100 + rand() % 156, 100 + rand() % 156);
}

int main() {
    // --- 1. CONFIGURACIÓN INICIAL ---

    // Configuración de la ventana de SFML
    const int WINDOW_WIDTH = 1280;
    const int WINDOW_HEIGHT = 720;
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Motor de Fisica 2D (Cajas y Circulos)");
    window.setFramerateLimit(60);

    // Creación del motor de física
    PhysicsEngine physicsEngine;

    // Ajustamos la gravedad
    physicsEngine.simulationPhysicsSettings.gravity = {0.0f, 250.f};

    // --- 2. CREACIÓN DE LOS OBJETOS FÍSICOS ---

    // Añadir un suelo estático (usando un Half-Space)
    // Un plano con normal hacia arriba (0, -1) posicionado en y=700
    physicsEngine.addHalfSpaceStaticObject({WINDOW_WIDTH / 2.0f, (float)WINDOW_HEIGHT - 20.f}, {0.0f, -1.0f});

    // Añadir una pared estática a la izquierda
    physicsEngine.addHalfSpaceStaticObject({20.f, WINDOW_HEIGHT / 2.0f}, {1.0f, 0.0f});

    // Añadir una pared estática a la derecha
    physicsEngine.addHalfSpaceStaticObject({(float)WINDOW_WIDTH - 20.f, WINDOW_HEIGHT / 2.0f}, {-1.0f, 0.0f});


    // --- Crear una pequeña pila de cajas ---
    for(int i = 0; i < 5; ++i) {
        auto boxId = physicsEngine.addBody({600.f, (float)(WINDOW_HEIGHT - 50 - i * 50)}, createBoxCollider({50.f, 50.f}));
        physicsEngine.bodies[boxId].elasticity = 0.1f;
        physicsEngine.bodies[boxId].staticFriction = 0.8f;
    }

    // Añadir un círculo para que choque contra la pila
    auto circleId = physicsEngine.addBody({450.f, (float)WINDOW_HEIGHT - 50}, createCircleCollider(25.f));
    physicsEngine.bodies[circleId].elasticity = 0.4f;
    physicsEngine.bodies[circleId].motionState.velocity = {150.f, 0.f}; // Dale un impulso inicial


    // Mapa para asociar IDs de cuerpos con formas de SFML
    std::unordered_map<ph2dBodyId, sf::Shape*> bodyShapes;

    // Formas visuales para el suelo y las paredes
    sf::RectangleShape floorShape, leftWallShape, rightWallShape;
    floorShape.setSize({(float)WINDOW_WIDTH, 20.f});
    floorShape.setPosition({0, (float)WINDOW_HEIGHT - 20.f});
    floorShape.setFillColor(sf::Color(180, 180, 180));

    leftWallShape.setSize({20.f, (float)WINDOW_HEIGHT});
    leftWallShape.setPosition({0, 0});
    leftWallShape.setFillColor(sf::Color(180, 180, 180));

    rightWallShape.setSize({20.f, (float)WINDOW_HEIGHT});
    rightWallShape.setPosition({(float)WINDOW_WIDTH - 20.f, 0});
    rightWallShape.setFillColor(sf::Color(180, 180, 180));

    // Reloj para medir el tiempo delta (deltaTime)
    sf::Clock deltaClock;

    // --- 3. BUCLE PRINCIPAL DEL JUEGO ---
    while (window.isOpen()) {
        // --- Manejo de eventos ---
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            // ¡Función extra! Haz clic para añadir un objeto nuevo.
            if (event.type == sf::Event::MouseButtonPressed) {
                Vec2 mousePos = {(float)event.mouseButton.x, (float)event.mouseButton.y};
                if (event.mouseButton.button == sf::Mouse::Left) {
                    // Clic izquierdo añade una caja
                    physicsEngine.addBody(mousePos, createBoxCollider({30.f, 30.f}));
                } else if (event.mouseButton.button == sf::Mouse::Right) {
                    // Clic derecho añade un círculo
                    physicsEngine.addBody(mousePos, createCircleCollider(20.f));
                }
            }
        }

        // --- Actualización de la física ---
        float deltaTime = deltaClock.restart().asSeconds();
        physicsEngine.runSimulation(deltaTime);

        // --- Renderizado ---
        window.clear(sf::Color(30, 30, 50)); // Fondo azul oscuro

        // Dibuja el suelo y las paredes
        window.draw(floorShape);
        window.draw(leftWallShape);
        window.draw(rightWallShape);

        // Dibuja cada cuerpo del motor
        for (const auto& pair : physicsEngine.bodies) {
            const ph2dBodyId id = pair.first;
            const Body& body = pair.second;

            // No dibujamos los semiplanos, ya que son infinitos
            if (body.isHalfPlane()) continue;

            sf::Shape* shape = nullptr;

            // Si es un cuerpo nuevo, crea su forma visual correspondiente
            if (bodyShapes.find(id) == bodyShapes.end()) {
                if (body.collider.type == ColliderBox) {
                    auto* rect = new sf::RectangleShape();
                    rect->setSize(toSfmlVec(body.collider.collider.box.size));
                    rect->setOrigin(rect->getSize().x / 2.f, rect->getSize().y / 2.f);
                    shape = rect;
                } else if (body.collider.type == ColliderCircle) {
                    auto* circle = new sf::CircleShape();
                    circle->setRadius(body.collider.collider.circle.radius);
                    circle->setOrigin(circle->getRadius(), circle->getRadius());
                    shape = circle;
                }

                if (shape) {
                    shape->setFillColor(getRandomColor());
                    shape->setOutlineThickness(1.5f);
                    shape->setOutlineColor(sf::Color(230, 230, 230));
                    bodyShapes[id] = shape;
                }
            }

            // Actualiza la posición y rotación de la forma visual
            shape = bodyShapes[id];
            if (shape) {
                shape->setPosition(toSfmlVec(body.motionState.pos));
                // Tu motor usa radianes, SFML usa grados. ¡Hay que convertir!
                shape->setRotation(Math::degrees(body.motionState.rotation));
                window.draw(*shape);
            }
        }

        window.display();
    }

    // --- Limpieza ---
    for (auto& pair : bodyShapes) {
        delete pair.second;
    }
    bodyShapes.clear();

    return 0;
}