#include <SFML/Graphics.hpp>
#include "polyphysics.h" // Asegúrate de que este es el nombre de tu header
#include <iostream>
#include <vector>
#include <unordered_map>

// --- FUNCIONES DE AYUDA ---
sf::Vector2f toSfmlVec(const Vec2& v) {
    return sf::Vector2f(v.x, v.y);
}
Vec2 fromSfmlVec(const sf::Vector2f& v) {
    return Vec2(v.x, v.y);
}

// --- VARIABLES GLOBALES PARA LA LÓGICA DEL JUEGO ---
PhysicsEngine physicsEngine;
ph2dBodyId birdId = 0;
std::vector<ph2dBodyId> structureBodyIds;
std::unordered_map<ph2dBodyId, sf::Shape*> bodyShapes;

const int WINDOW_WIDTH = 1280;
const int WINDOW_HEIGHT = 720;

// --- FUNCIÓN PARA REINICIAR LA ESCENA ---
void resetScene() {
    // Limpiar el motor y las formas visuales
    physicsEngine.bodies.clear();
    for (auto& pair : bodyShapes) {
        delete pair.second;
    }
    bodyShapes.clear();
    structureBodyIds.clear();

    // --- Crear el mundo ---
    // Suelo
    physicsEngine.addHalfSpaceStaticObject({WINDOW_WIDTH / 2.0f, (float)WINDOW_HEIGHT - 20.f}, {0.0f, -1.0f});
    // Pared derecha (para que no se caiga todo)
    physicsEngine.addHalfSpaceStaticObject({(float)WINDOW_WIDTH - 20.f, WINDOW_HEIGHT / 2.0f}, {-1.0f, 0.0f});

    // --- Crear la estructura de cajas ---
    // Base
    auto boxId1 = physicsEngine.addBody({900.f, (float)WINDOW_HEIGHT - 50.f}, createBoxCollider({150.f, 25.f}));
    auto boxId2 = physicsEngine.addBody({1050.f, (float)WINDOW_HEIGHT - 50.f}, createBoxCollider({150.f, 25.f}));
    // Nivel medio
    auto boxId3 = physicsEngine.addBody({975.f, (float)WINDOW_HEIGHT - 100.f}, createBoxCollider({25.f, 50.f}));
    auto boxId4 = physicsEngine.addBody({900.f, (float)WINDOW_HEIGHT - 150.f}, createBoxCollider({150.f, 25.f}));
    auto boxId5 = physicsEngine.addBody({1050.f, (float)WINDOW_HEIGHT - 150.f}, createBoxCollider({150.f, 25.f}));
    // Nivel superior
    auto boxId6 = physicsEngine.addBody({975.f, (float)WINDOW_HEIGHT - 200.f}, createBoxCollider({25.f, 50.f}));

    structureBodyIds = {boxId1, boxId2, boxId3, boxId4, boxId5, boxId6};

    // --- HACER QUE LA ESTRUCTURA SEA FIJA INICIALMENTE ---
    for (auto id : structureBodyIds) {
        physicsEngine.bodies[id].flags.setKinematic(true);
        physicsEngine.bodies[id].elasticity = 0.1f;
        physicsEngine.bodies[id].staticFriction = 0.2f;
        physicsEngine.bodies[id].dynamicFriction = 0.2f;
    }

    // --- Crear el "Pájaro" ---
    birdId = physicsEngine.addBody({200.f, 550.f}, createCircleCollider(25.f));
    physicsEngine.bodies[birdId].elasticity = 0.5f;
    physicsEngine.bodies[birdId].motionState.mass = 20.f; // Hacemos al pájaro más pesado
    physicsEngine.bodies[birdId].flags.setKinematic(true); // Lo hacemos cinemático hasta que se lance
}


int main() {
    // --- CONFIGURACIÓN INICIAL ---
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulacion Estilo Angry Birds");
    window.setFramerateLimit(120);

    physicsEngine.simulationPhysicsSettings.gravity = {0.0f, 400.0f};
    physicsEngine.collisionChecksCount = 24; // Más iteraciones para pilas estables
    physicsEngine.simulationPhysicsSettings.airDragCoefficient = 0.0005f;
    physicsEngine.simulationPhysicsSettings.rotationalDragCoefficient = 0.0002f;
    resetScene();

    // Variables para el lanzamiento
    bool isDragging = false;
    sf::Vector2i dragStartPos;
    sf::Vertex line[2];

    // Formas visuales para el suelo y las paredes
    sf::RectangleShape floorShape, rightWallShape;
    floorShape.setSize({(float)WINDOW_WIDTH, 20.f});
    floorShape.setPosition({0, (float)WINDOW_HEIGHT - 20.f});
    floorShape.setFillColor(sf::Color(100, 80, 70));
    rightWallShape.setSize({20.f, (float)WINDOW_HEIGHT});
    rightWallShape.setPosition({(float)WINDOW_WIDTH - 20.f, 0});
    rightWallShape.setFillColor(sf::Color(100, 80, 70));

    sf::Clock deltaClock;

    // --- BUCLE PRINCIPAL DEL JUEGO ---
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
            if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::R) {
                resetScene();
            }

            // --- Lógica de lanzamiento ---
            if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
                 if (physicsEngine.bodies[birdId].flags.isKinematic()) {
                    isDragging = true;
                    dragStartPos = sf::Mouse::getPosition(window);
                 }
            }
            if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
                if (isDragging) {
                    isDragging = false;
                    sf::Vector2i dragEndPos = sf::Mouse::getPosition(window);
                    sf::Vector2f dragVector = sf::Vector2f(dragStartPos - dragEndPos);

                    physicsEngine.bodies[birdId].flags.setKinematic(false);
                    physicsEngine.bodies[birdId].motionState.velocity = fromSfmlVec(dragVector) * 2.f;
                }
            }
        }

        // --- ACTUALIZACIÓN DE LA FÍSICA ---
        float deltaTime = deltaClock.restart().asSeconds();
        physicsEngine.runSimulation(deltaTime);

        // --- LÓGICA DEL JUEGO (POST-FÍSICA) ---
        // Implementación de la reacción en cadena

        std::vector<ph2dBodyId> bodiesToWakeUp;
        for (const auto& manifold : physicsEngine.intersections) {
            Body& bodyA = physicsEngine.bodies[manifold.A];
            Body& bodyB = physicsEngine.bodies[manifold.B];

            // Si un cuerpo dinámico choca con uno cinemático de la estructura, lo despertamos.
            if (!bodyA.flags.isKinematic() && bodyB.flags.isKinematic()) {
                for (auto structureId : structureBodyIds) {
                    if (manifold.B == structureId) {
                        bodiesToWakeUp.push_back(manifold.B);
                        break;
                    }
                }
            }
            if (!bodyB.flags.isKinematic() && bodyA.flags.isKinematic()) {
                for (auto structureId : structureBodyIds) {
                    if (manifold.A == structureId) {
                        bodiesToWakeUp.push_back(manifold.A);
                        break;
                    }
                }
            }
        }

        // Activamos todos los cuerpos que deben despertar
        for (auto id : bodiesToWakeUp) {
            physicsEngine.bodies[id].flags.setKinematic(false);
        }

        // --- RENDERIZADO ---
        window.clear(sf::Color(40, 45, 60));

        window.draw(floorShape);
        window.draw(rightWallShape);

        // Dibuja cada cuerpo del motor
        for (const auto& pair : physicsEngine.bodies) {
            const ph2dBodyId id = pair.first;
            const Body& body = pair.second;
            if (body.isHalfPlane()) continue;

            sf::Shape* shape = nullptr;
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
                    bodyShapes[id] = shape;
                    if (id == birdId) {
                        shape->setFillColor(sf::Color(255, 80, 80));
                    } else {
                         shape->setFillColor(sf::Color(200, 180, 150));
                    }
                    shape->setOutlineThickness(2.f);
                    shape->setOutlineColor(sf::Color(80, 60, 50));
                }
            }

            shape = bodyShapes[id];
            if (shape) {
                shape->setPosition(toSfmlVec(body.motionState.pos));
                shape->setRotation(Math::degrees(body.motionState.rotation));
                window.draw(*shape);
            }
        }

        // Dibujar la línea de lanzamiento
        if (isDragging) {
            sf::Vector2f birdPos = toSfmlVec(physicsEngine.bodies[birdId].motionState.pos);
            line[0] = sf::Vertex(birdPos, sf::Color::White);
            sf::Vector2f mousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
            sf::Vector2f dragVec = birdPos - mousePos;
            line[1] = sf::Vertex(birdPos + dragVec, sf::Color::Red);
            window.draw(line, 2, sf::Lines);
        }

        window.display();
    }

    // --- LIMPIEZA ---
    for (auto& pair : bodyShapes) {
        delete pair.second;
    }

    return 0;
}