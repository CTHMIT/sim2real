import os
import sys

print(f"CUDA_HOME: {os.environ.get('CUDA_HOME', 'Not set')}")
print(f"LD_LIBRARY_PATH: {os.environ.get('LD_LIBRARY_PATH', 'Not set')}")
print(f"Python version: {sys.version}")
print(f"Python executable: {sys.executable}")

# 先測試基本的 CUDA 功能
try:
    import pycuda.driver as cuda

    print("成功導入 pycuda.driver")

    # 初始化 CUDA
    try:
        cuda.init()
        print("CUDA 初始化成功")
    except Exception as e:
        print(f"CUDA 初始化失敗: {e}")

    print(f"CUDA 驅動版本: {cuda.get_version()}")
    print(f"CUDA 設備數量: {cuda.Device.count()}")

    # 獲取第一個 GPU 設備的信息
    device = cuda.Device(0)
    print(f"使用設備: {device.name()}")
    print(f"計算能力: {device.compute_capability()}")
    print(f"總內存: {device.total_memory() // (1024**2)} MB")

    # 測試 PyCUDA GL 模塊
    try:
        from pycuda.gl import graphics_map_flags

        print("成功導入 pycuda.gl.graphics_map_flags")
    except ImportError as e:
        print(f"導入 pycuda.gl.graphics_map_flags 失敗: {e}")

    CUDA_AVAILABLE = True

except ImportError as e:
    print(f"導入 PyCUDA 失敗: {e}")
    CUDA_AVAILABLE = False
except Exception as e:
    print(f"PyCUDA 使用過程中出錯: {e}")
    CUDA_AVAILABLE = False

# 測試 OpenGL
try:
    import OpenGL.GL as gl
    import glfw

    OPENGL_AVAILABLE = True
    print("OpenGL 可用")
except ImportError as e:
    OPENGL_AVAILABLE = False
    print(f"OpenGL 不可用: {e}")

# CUDA-OpenGL 互操作測試
if CUDA_AVAILABLE and OPENGL_AVAILABLE:
    print("\n開始 CUDA-OpenGL 互操作測試...")
    try:
        # 初始化 GLFW
        if not glfw.init():
            print("無法初始化 GLFW")
        else:
            print("GLFW 初始化成功")

            # 創建視窗
            window = glfw.create_window(640, 480, "CUDA-OpenGL 測試", None, None)
            if not window:
                print("無法創建 GLFW 視窗")
                glfw.terminate()
            else:
                print("GLFW 視窗創建成功")
                glfw.make_context_current(window)

                # 創建 OpenGL 紋理
                width, height = 640, 480
                gl_texture = gl.glGenTextures(1)
                gl.glBindTexture(gl.GL_TEXTURE_2D, gl_texture)
                gl.glTexImage2D(
                    gl.GL_TEXTURE_2D,
                    0,
                    gl.GL_RGBA,
                    width,
                    height,
                    0,
                    gl.GL_RGBA,
                    gl.GL_UNSIGNED_BYTE,
                    None,
                )

                # 測試 CUDA-OpenGL 互操作
                try:
                    from pycuda.gl import graphics_map_flags

                    cuda_graphics_resource = cuda.GraphicsGLRegisterImage(
                        gl_texture, gl.GL_TEXTURE_2D, graphics_map_flags.WRITE_DISCARD
                    )
                    print("CUDA-OpenGL 互操作功能測試成功")
                except Exception as e:
                    print(f"CUDA-OpenGL 互操作測試失敗: {e}")

                glfw.terminate()
    except Exception as e:
        print(f"測試過程中發生錯誤: {e}")
else:
    if not CUDA_AVAILABLE:
        print("由於 CUDA 不可用，跳過 CUDA-OpenGL 互操作測試")
    if not OPENGL_AVAILABLE:
        print("由於 OpenGL 不可用，跳過 CUDA-OpenGL 互操作測試")
