#pragma once

#include <cstdint>
#include <functional>
#include <string>

namespace blit {
    // misc structs/enums
    struct ButtonState {
        uint32_t state;
        uint32_t pressed, released;
    };

    struct FileInfo {
        std::string name;
        int flags;
        uint32_t size;
    };

    struct GameMetadata {
        const char *title = nullptr;
        const char *author = nullptr;
        const char *description = nullptr;
        const char *version = nullptr;
        const char *url = nullptr;
        const char *category = nullptr;
    };

    enum class PixelFormat {
        RGB = 0,
        RGBA = 1,
        P = 2,
        M = 3,
        RGB565 = 4
    };

    #pragma pack(push, 1)
    struct alignas(4) Pen {
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        uint8_t a = 0;
    };
    #pragma pack(pop)

    struct Rect {
        int32_t x, y, w, h;
    };

    enum ScreenMode {
        lores, hires, hires_palette
    };

    struct Size {
        int32_t w, h;
    };

    struct JPEGImage {
        Size size;
        uint8_t *data;
    };

    struct Vec2 {
        float x;
        float y;
    };

    struct Vec3 {
        float x;
        float y;
        float z;
    };

    // 32blit/engine/api_private.hpp

    using AllocateCallback = uint8_t *(*)(size_t);

    constexpr uint16_t api_version_major = 0, api_version_minor = 1;

    // template for screen modes
    struct SurfaceTemplate {
        uint8_t *data = nullptr;
        Size bounds;
        PixelFormat format;
        Pen *palette = nullptr;
    };

    // subset of Surface for API compat
    struct SurfaceInfo {
        SurfaceInfo() = default;
        SurfaceInfo(const SurfaceTemplate &surf): data(surf.data), bounds(surf.bounds), format(surf.format), palette(surf.palette) {}

        uint8_t *data = nullptr;
        Size bounds;

        // unused, here for compat reasons
        Rect clip;
        uint8_t alpha;
        Pen pen;

        PixelFormat format;
        uint8_t pixel_stride; // unused
        uint16_t row_stride; // unused

        void *mask = nullptr; // unused
        Pen *palette = nullptr;
    };

    #pragma pack(push, 4)
    struct API {
        uint16_t version_major;
        uint16_t version_minor;

        ButtonState buttons;
        float hack_left;
        float hack_right;
        float vibration;
        Vec2 joystick;
        Vec3 tilt;
        Pen LED;

        /*AudioChannel*/void *channels; // TODO

        uint32_t set_screen_mode;
        uint32_t set_screen_palette;
        uint32_t now;
        uint32_t random;
        uint32_t exit;

        // serial debug
        uint32_t debug;

        // files
        uint32_t open_file;
        uint32_t read_file;
        uint32_t write_file;
        uint32_t close_file;
        uint32_t get_file_length;
        uint32_t list_files;
        uint32_t file_exists;
        uint32_t directory_exists;
        uint32_t create_directory;
        uint32_t rename_file;
        uint32_t remove_file;
        uint32_t get_save_path;
        uint32_t is_storage_available;

        // profiler
        uint32_t enable_us_timer;
        uint32_t get_us_timer;
        uint32_t get_max_us_timer;

        // jepg
        uint32_t decode_jpeg_buffer;
        uint32_t decode_jpeg_file;

        // launcher APIs - only intended for use by launchers and only available on device
        uint32_t launch;
        uint32_t erase_game;
        uint32_t get_type_handler_metadata;

        uint32_t get_launch_path;

        // multiplayer
        uint32_t is_multiplayer_connected;
        uint32_t set_multiplayer_enabled;
        uint32_t send_message;
        uint32_t message_received; // set by user

        uint32_t flash_to_tmp;
        uint32_t tmp_file_closed;

        uint32_t get_metadata;

        bool tick_function_changed;

        uint32_t set_screen_mode_format;

        // raw i2c access
        uint32_t i2c_send;
        uint32_t i2c_receive;
        uint32_t i2c_completed; // callback when done

        // raw cdc
        uint32_t set_raw_cdc_enabled;
        uint32_t cdc_write;
        uint32_t cdc_read;
    };
    #pragma pack(pop)
}