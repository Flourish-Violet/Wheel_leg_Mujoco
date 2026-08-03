#include "viewer_glfw.h"

#include <stdio.h>
#include <windows.h>

typedef struct GLFWwindow GLFWwindow;

typedef int (*GlfwInitFn)(void);
typedef void (*GlfwTerminateFn)(void);
typedef GLFWwindow* (*GlfwCreateWindowFn)(int, int, const char*, void*, void*);
typedef void (*GlfwDestroyWindowFn)(GLFWwindow*);
typedef void (*GlfwMakeContextCurrentFn)(GLFWwindow*);
typedef void (*GlfwSwapIntervalFn)(int);
typedef int (*GlfwWindowShouldCloseFn)(GLFWwindow*);
typedef void (*GlfwPollEventsFn)(void);
typedef void (*GlfwSwapBuffersFn)(GLFWwindow*);
typedef void (*GlfwGetFramebufferSizeFn)(GLFWwindow*, int*, int*);

struct GlfwApi {
    HMODULE dll;
    GlfwInitFn init;
    GlfwTerminateFn terminate;
    GlfwCreateWindowFn create_window;
    GlfwDestroyWindowFn destroy_window;
    GlfwMakeContextCurrentFn make_context_current;
    GlfwSwapIntervalFn swap_interval;
    GlfwWindowShouldCloseFn window_should_close;
    GlfwPollEventsFn poll_events;
    GlfwSwapBuffersFn swap_buffers;
    GlfwGetFramebufferSizeFn get_framebuffer_size;
};

static FARPROC load_symbol(HMODULE dll, const char* name) {
    FARPROC proc = GetProcAddress(dll, name);
    if (!proc) {
        printf("missing GLFW symbol: %s\n", name);
    }
    return proc;
}

static int load_glfw(GlfwApi* g) {
    g->dll = LoadLibraryA("glfw3.dll");
    if (!g->dll) {
        g->dll = LoadLibraryA(
            "D:/APP/Mujoco/MujocoRef/wheel_legged_robot_sim/.venv/Lib/site-packages/glfw/glfw3.dll");
    }
    if (!g->dll) {
        printf("failed to load glfw3.dll\n");
        return 0;
    }

    g->init = (GlfwInitFn)load_symbol(g->dll, "glfwInit");
    g->terminate = (GlfwTerminateFn)load_symbol(g->dll, "glfwTerminate");
    g->create_window = (GlfwCreateWindowFn)load_symbol(g->dll, "glfwCreateWindow");
    g->destroy_window = (GlfwDestroyWindowFn)load_symbol(g->dll, "glfwDestroyWindow");
    g->make_context_current = (GlfwMakeContextCurrentFn)load_symbol(g->dll, "glfwMakeContextCurrent");
    g->swap_interval = (GlfwSwapIntervalFn)load_symbol(g->dll, "glfwSwapInterval");
    g->window_should_close = (GlfwWindowShouldCloseFn)load_symbol(g->dll, "glfwWindowShouldClose");
    g->poll_events = (GlfwPollEventsFn)load_symbol(g->dll, "glfwPollEvents");
    g->swap_buffers = (GlfwSwapBuffersFn)load_symbol(g->dll, "glfwSwapBuffers");
    g->get_framebuffer_size = (GlfwGetFramebufferSizeFn)load_symbol(g->dll, "glfwGetFramebufferSize");

    return g->init && g->terminate && g->create_window && g->destroy_window &&
           g->make_context_current && g->swap_interval && g->window_should_close &&
           g->poll_events && g->swap_buffers && g->get_framebuffer_size;
}

int run_mujoco_viewer(mjModel* m, mjData* d, ControlStepFn control_step, void* user) {
    GlfwApi glfw = {};
    if (!load_glfw(&glfw)) {
        return 1;
    }
    if (!glfw.init()) {
        printf("glfwInit failed\n");
        return 1;
    }

    GLFWwindow* window = glfw.create_window(1200, 900, "wheel_leg_cpp_control", 0, 0);
    if (!window) {
        printf("glfwCreateWindow failed\n");
        glfw.terminate();
        return 1;
    }

    glfw.make_context_current(window);
    glfw.swap_interval(1);

    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    cam.azimuth = 90.0;
    cam.elevation = -20.0;
    cam.distance = 2.5;
    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.25;

    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    while (!glfw.window_should_close(window)) {
        double frame_start = d->time;
        while (d->time - frame_start < 1.0 / 60.0) {
            if (control_step) {
                control_step(m, d, user);
            }
            mj_step(m, d);
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfw.get_framebuffer_size(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, 0, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfw.swap_buffers(window);
        glfw.poll_events();
    }

    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfw.destroy_window(window);
    glfw.terminate();
    return 0;
}
