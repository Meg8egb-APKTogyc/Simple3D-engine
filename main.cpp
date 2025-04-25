#include "headers/Camera.hpp"
#include "headers/Mesh.hpp"
#include <chrono>
#include <X11/X.h>
#include <thread>
#include <cmath>


int main() {
    // Инициализация графического контекста
    const int width = 800;
    const int height = 600;
    GraphicsContext gc(width, height, "3D Torus with Camera Control");

    // Настройка камеры (ближе к тору для лучшего обзора)
    Vector3D cameraPos(0.0, 0.0, 5.0);
    Vector3D target(0.0, 0.0, 0.0);
    Vector3D up(0.0, 1.0, 0.0);
    Camera camera(cameraPos, target, up, -1.0, 1.0, -1.0, 1.0, 1.0, 100.0);
    camera.setViewport(width, height);
    camera.setMoveSpeed(3.0); // Увеличим скорость движения
    camera.setSensivity(0.05); // Средняя чувствительность мыши

    // Настройка освещения
    Light light(Vector3D(30.0, 30.0, 30.0), Vector3D(1.0, 1.0, 1.0), 1.0, true);
    Render render(camera, gc, light);
    render.setAmbientColor(Vector3D(0.3, 0.3, 0.3)); // Усилим ambient для лучшей видимости

    // Создание и генерация тора
    ParametricTorus torus;
    torus.generatePolygons(1.0, 0.3); // Основной радиус 1.0, радиус трубы 0.3

    // Переменные для управления
    bool keys[256] = {false};
    auto lastFrame = std::chrono::high_resolution_clock::now();
    bool running = true;

    bool mouseCaptured = false;
    Display* display = gc.getDisplay();
    Window rootWindow = DefaultRootWindow(display);

    // Основной цикл рендеринга
    while (running) {
        // Расчет времени между кадрами
        auto currentFrame = std::chrono::high_resolution_clock::now();
        float deltaTime = std::chrono::duration<float>(currentFrame - lastFrame).count();
        lastFrame = currentFrame;

        // Обработка событий
        XEvent event;
        while (XPending(gc.getDisplay())) {
            XNextEvent(gc.getDisplay(), &event);
            
            if (event.type == KeyPress) {
                KeyCode keycode = event.xkey.keycode;
                keys[keycode] = (event.type == KeyPress);

                KeySym key = XLookupKeysym(&event.xkey, 0);
                if (key == XK_Escape) {
                    running = false;
                    continue;
                }

                if (key == XK_space) { // Пробел для захвата/освобождения мыши
                    mouseCaptured = !mouseCaptured;
                    if (mouseCaptured) {
                        // Захватываем мышь
                        int result = XGrabPointer(
                            gc.getDisplay(),     // Дисплей
                            gc.getWindow(),      // Наше окно
                            True,                // Владеем событиями
                            PointerMotionMask,   // Следим за движением
                            GrabModeAsync,       // Асинхронный режим
                            GrabModeAsync,       // Асинхронный режим
                            rootWindow,          // Корневое окно
                            None,                // Без специального курсора
                            CurrentTime          // Текущее время
                        );

                        Cursor invisibleCursor;
                        Pixmap bitmapNoData = XCreateBitmapFromData(display, gc.getWindow(), "", 1, 1);
                        XColor black;
                        black.red = black.green = black.blue = 0;
                        invisibleCursor = XCreatePixmapCursor(display, bitmapNoData, bitmapNoData,
                                                            &black, &black, 0, 0);
                        XDefineCursor(display, gc.getWindow(), invisibleCursor);
                        XFreeCursor(display, invisibleCursor);
                        XFreePixmap(display, bitmapNoData);

                        if (result != GrabSuccess) {
                            std::cerr << "Не удалось захватить указатель" << std::endl;
                            mouseCaptured = false;
                        }
                    } else {
                        // Освобождаем указатель
                        XUngrabPointer(display, CurrentTime);
                        XUndefineCursor(display, gc.getWindow());
                    }
                } else if (key == XK_plus || key == XK_KP_Add) {
                    // Увеличить малый радиус
                    torus.setTubeRadius(torus.getTubeRadius() + 0.1);
                } else if (key == XK_minus || key == XK_KP_Subtract) {
                    // Уменьшить малый радиус с ограничением
                    torus.setTubeRadius(std::max(0.1, torus.getTubeRadius() - 0.1));
                } else if (key == XK_bracketleft) {
                    // Увеличить большой радиус
                    torus.setMajorRadius(torus.getMajorRadius() + 0.1);
                } else if (key == XK_bracketright) {
                    // Уменьшить большой радиус
                    torus.setMajorRadius(std::max(0.5, torus.getMajorRadius() - 0.1));
                }
            }
            else if (event.type == KeyRelease) {
                KeyCode keycode = event.xkey.keycode;
                keys[keycode] = (event.type == KeyPress);
            } else if (mouseCaptured && event.type == MotionNotify) {
                int centerX = width / 2;
                int centerY = height / 2;
                
                float xoffset = event.xmotion.x - centerX;
                float yoffset = centerY - event.xmotion.y; // Инвертируем Y

                // Ограничиваем максимальное смещение
                const float maxOffset = 100.0f;
                xoffset = std::clamp(xoffset, -maxOffset, maxOffset);
                yoffset = std::clamp(yoffset, -maxOffset, maxOffset);
                
                // Применяем нелинейное преобразование для более точного управления
                double sign1 = xoffset >= 0 ? 1.0 : -1.0;
                double sign2 = yoffset >= 0 ? 1.0 : -1.0;
                xoffset = sign1 * pow(fabs(xoffset)/maxOffset, 2.0) * maxOffset;
                yoffset = sign2 * pow(fabs(yoffset)/maxOffset, 2.0) * maxOffset;

                if (fabs(xoffset) > 2 || fabs(yoffset) > 2) {
                    // Возвращаем курсор в центр
                    XWarpPointer(display, None, gc.getWindow(), 
                               0, 0, 0, 0, centerX, centerY);
                    
                    camera.processMouseMovement(xoffset, yoffset);
                }
            } else if (event.type == ButtonPress) { // Колесо вверх
                if (event.xbutton.button == 4) { // Колесо вверх
                    camera.processMouseScroll(1.0);
                } else if (event.xbutton.button == 5) { // Колесо вниз
                    camera.processMouseScroll(-1.0);
                }
            }
        }

        // Обработка клавиш движения
        KeyCode w_keycode = XKeysymToKeycode(display, XK_w);
        KeyCode s_keycode = XKeysymToKeycode(display, XK_s);
        KeyCode a_keycode = XKeysymToKeycode(display, XK_a);
        KeyCode d_keycode = XKeysymToKeycode(display, XK_d);
        KeyCode q_keycode = XKeysymToKeycode(display, XK_q);
        KeyCode e_keycode = XKeysymToKeycode(display, XK_e);
        
        if (keys[w_keycode]) camera.processMovement(XK_w, deltaTime);
        if (keys[s_keycode]) camera.processMovement(XK_s, deltaTime);
        if (keys[a_keycode]) camera.processMovement(XK_a, deltaTime);
        if (keys[d_keycode]) camera.processMovement(XK_d, deltaTime);
        if (keys[q_keycode]) camera.processMovement(XK_q, deltaTime);
        if (keys[e_keycode]) camera.processMovement(XK_e, deltaTime);

        // Очистка и рендеринг
        gc.clear(0xFF222222); // Темно-серый фон
        
        // Применяем небольшую анимацию тора (вращение)
        static float angle = 0.1f;
        Matrix4x4 model = Matrix4x4::rotateY(angle) * Matrix4x4::translate(0.0, 0.0, 0.0);
        torus.applyTransform(model);
        
        // Рендерим тор с подсветкой и гранями
        render.drawMesh3D(torus, 0xFF0000AA, false, true);

        std::string info = "Torus R: " + std::to_string(torus.getMajorRadius()) + 
                   " r: " + std::to_string(torus.getTubeRadius()) + 
                   " (+, - to change)";
        XStoreName(display, gc.getWindow(), info.c_str());

        // Обновление экрана
        gc.flush();

        std::this_thread::sleep_for(std::chrono::milliseconds(16)); 
    }

    if (mouseCaptured) {
        XUngrabPointer(display, CurrentTime);
        XUndefineCursor(display, gc.getWindow());
    }
    XFlush(display);

    return 0;
}