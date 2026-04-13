// common/util/include/util/plugin_loader.h
// Runtime .so plugin loading via dlopen/dlsym/dlclose.
//
// RAII wrapper that ensures correct lifetime ordering: the dlopen handle
// outlives all objects created from the loaded .so.  If the handle is
// dlclose'd before a plugin object is destroyed, the vtable pointers
// become dangling => use-after-free.
//
// Design:
//   PluginHandle<I>  — owns both the dl handle and the instance.
//                      Destruction: instance first, then dlclose.
//   PluginRegistry   — process-lifetime storage for dl handles when
//                      the caller needs a unique_ptr<Interface> (e.g.
//                      HAL factory).  Keeps handles alive until exit.
//   PluginLoader     — static factory: load() opens a .so, resolves
//                      a C-linkage factory symbol, creates an instance.
//
// Plugin .so requirements:
//   - Factory function MUST use extern "C" linkage to avoid name mangling.
//   - Factory function MUST have __attribute__((visibility("default"))).
//   - Factory signature: Interface* symbol_name();
//   - The caller takes ownership of the returned raw pointer.
//
// Security:
//   ENABLE_PLUGINS should only be enabled in trusted environments.
//   Arbitrary .so loading can execute any code.  The path is validated
//   to be a regular file before dlopen.
//
// Thread safety:
//   Plugin loading should happen at startup only.  dlopen is thread-safe
//   on glibc, but loaded code's static initializers run synchronously.
//   PluginRegistry::instance() is initialized thread-safely (C++11 magic
//   statics) but store() is NOT thread-safe — call only from main thread.
//
// Gated behind: #ifdef HAVE_PLUGINS (cmake -DENABLE_PLUGINS=ON)
#pragma once

#ifdef HAVE_PLUGINS

#include "util/ilogger.h"
#include "util/result.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <dlfcn.h>
#include <sys/stat.h>

namespace drone::util {

// ── PluginHandle ───────────────────────────────────────────
/// RAII handle owning a dlopen'd .so AND the instance created from it.
/// Destruction order: instance_.reset() first, then dlclose(dl_handle_).
/// Move-only.
template<typename Interface>
class PluginHandle {
public:
    ~PluginHandle() noexcept {
        // Order matters: destroy instance before closing the library.
        instance_.reset();
        if (dl_handle_) {
            ::dlclose(dl_handle_);
            dl_handle_ = nullptr;
        }
    }

    PluginHandle(PluginHandle&& other) noexcept
        : dl_handle_(other.dl_handle_), instance_(std::move(other.instance_)) {
        other.dl_handle_ = nullptr;
    }

    PluginHandle& operator=(PluginHandle&& other) noexcept {
        if (this != &other) {
            // Clean up current state
            instance_.reset();
            if (dl_handle_) {
                ::dlclose(dl_handle_);
            }
            // Take ownership
            dl_handle_       = other.dl_handle_;
            instance_        = std::move(other.instance_);
            other.dl_handle_ = nullptr;
        }
        return *this;
    }

    PluginHandle(const PluginHandle&)            = delete;
    PluginHandle& operator=(const PluginHandle&) = delete;

    /// Access the plugin instance.  Returns nullptr if moved-from.
    [[nodiscard]] Interface* get() const noexcept { return instance_.get(); }

    /// Dereference.  Precondition: get() != nullptr.
    [[nodiscard]] Interface& operator*() const { return *instance_; }

    /// Arrow access.  Precondition: get() != nullptr.
    [[nodiscard]] Interface* operator->() const noexcept { return instance_.get(); }

    /// Release the instance as unique_ptr, transferring ownership to the caller.
    /// The dl_handle remains owned by this PluginHandle — caller must ensure
    /// this PluginHandle (or a PluginRegistry holding it) outlives the returned ptr.
    [[nodiscard]] std::unique_ptr<Interface> release_instance() noexcept {
        return std::move(instance_);
    }

private:
    friend class PluginLoader;
    friend class PluginRegistry;

    /// Release ONLY the dl_handle, returning it as a void*.
    /// Private — only PluginRegistry::extract() should call this to safely
    /// transfer the dl_handle to process-lifetime storage.  Calling dlclose()
    /// on the returned handle while a live Interface instance still holds
    /// vtable pointers from the library causes use-after-free.
    [[nodiscard]] void* release_dl_handle() noexcept {
        void* h    = dl_handle_;
        dl_handle_ = nullptr;
        return h;
    }

    PluginHandle(void* dl_handle, Interface* instance)
        : dl_handle_(dl_handle), instance_(instance) {}

    void*                      dl_handle_ = nullptr;
    std::unique_ptr<Interface> instance_;
};

// ── PluginRegistry ─────────────────────────────────────────
/// Process-lifetime storage for dl handles.
///
/// When HAL factories need to return unique_ptr<Interface> but the dl
/// handle must outlive the interface object, the registry stores the
/// handle.  dlclose happens only at process exit (static destructor).
///
/// IMPORTANT: All plugin-created instances (unique_ptr<Interface>) must be
/// destroyed BEFORE static destructors run.  In practice, this means plugin
/// instances should be held in main() locals or in objects owned by main(),
/// never in static/global variables.  If a plugin instance outlives
/// ~PluginRegistry, its vtable pointers will dangle (use-after-free).
///
/// Thread safety: NOT thread-safe.  Call store() from main thread only
/// during startup.
class PluginRegistry {
public:
    /// Meyer's singleton.
    static PluginRegistry& instance() {
        static PluginRegistry reg;
        return reg;
    }

    /// Store a dl_handle for process-lifetime retention.
    /// Returns nothing — the handle will be dlclose'd at process exit.
    void store_handle(void* dl_handle) {
        if (dl_handle) {
            handles_.push_back(dl_handle);
        }
    }

    /// Convenience: extract the instance from a PluginHandle as unique_ptr,
    /// and store the dl_handle in the registry for process-lifetime retention.
    /// This is the pattern HAL factories should use.
    template<typename Interface>
    [[nodiscard]] std::unique_ptr<Interface> extract(PluginHandle<Interface> handle) {
        auto instance = handle.release_instance();
        store_handle(handle.release_dl_handle());
        return instance;
    }

    /// Number of stored handles (for testing).
    [[nodiscard]] std::size_t handle_count() const noexcept { return handles_.size(); }

    // Non-copyable, non-movable.
    PluginRegistry(const PluginRegistry&)            = delete;
    PluginRegistry& operator=(const PluginRegistry&) = delete;
    PluginRegistry(PluginRegistry&&)                 = delete;
    PluginRegistry& operator=(PluginRegistry&&)      = delete;

private:
    PluginRegistry() = default;

    ~PluginRegistry() {
        // dlclose in reverse order (LIFO) for safety.
        for (auto it = handles_.rbegin(); it != handles_.rend(); ++it) {
            if (*it) {
                ::dlclose(*it);
            }
        }
        handles_.clear();
    }

    std::vector<void*> handles_;
};

// ── PluginLoader ───────────────────────────────────────────
/// Static factory for loading plugin .so files.
class PluginLoader {
public:
    /// Load a plugin .so and create an instance via the named factory function.
    ///
    /// The factory function in the .so must be:
    ///   extern "C" __attribute__((visibility("default")))
    ///   Interface* factory_symbol();
    ///
    /// @param so_path          Path to the .so file (must exist and be a regular file).
    /// @param factory_symbol   Name of the C-linkage factory function (default: "create_instance").
    /// @return PluginHandle owning both the dl handle and the created instance,
    ///         or an error string on failure.
    template<typename Interface>
    [[nodiscard]] static Result<PluginHandle<Interface>, std::string> load(
        const std::string& so_path, const std::string& factory_symbol = "create_instance") {
        // Validate path exists and is a regular file.
        struct stat st {};
        if (::stat(so_path.c_str(), &st) != 0) {
            return Result<PluginHandle<Interface>, std::string>::err("Plugin file not found: " +
                                                                     so_path);
        }
        if (!S_ISREG(st.st_mode)) {
            return Result<PluginHandle<Interface>, std::string>::err(
                "Plugin path is not a regular file: " + so_path);
        }

        DRONE_LOG_WARN("[PluginLoader] Loading plugin from '{}' — "
                       "ensure this .so is from a trusted source",
                       so_path);

        // Clear any stale dlerror state.
        ::dlerror();

        // Open the shared library.
        void* dl_handle = ::dlopen(so_path.c_str(), RTLD_NOW | RTLD_LOCAL);
        if (!dl_handle) {
            // Capture dlerror immediately — it's cleared by subsequent dl* calls.
            const char* err = ::dlerror();
            std::string msg = err ? std::string(err) : "dlopen failed (unknown error)";
            return Result<PluginHandle<Interface>, std::string>::err("Failed to load plugin '" +
                                                                     so_path + "': " + msg);
        }

        // Clear dlerror before dlsym.
        ::dlerror();

        // Resolve the factory symbol.
        void*       sym     = ::dlsym(dl_handle, factory_symbol.c_str());
        const char* sym_err = ::dlerror();
        if (sym_err || !sym) {
            std::string msg = sym_err ? std::string(sym_err)
                                      : "dlsym returned null for '" + factory_symbol + "'";
            ::dlclose(dl_handle);
            return Result<PluginHandle<Interface>, std::string>::err(
                "Failed to resolve symbol '" + factory_symbol + "' in '" + so_path + "': " + msg);
        }

        // Cast to factory function pointer.
        // The factory must return Interface* — the caller takes ownership.
        using FactoryFunc = Interface* (*)();
        // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast) — required for dlsym
        auto factory = reinterpret_cast<FactoryFunc>(sym);

        // Call the factory — guard against exceptions to avoid leaking dl_handle.
        Interface* instance = nullptr;
        try {
            instance = factory();
        } catch (const std::exception& e) {
            ::dlclose(dl_handle);
            return Result<PluginHandle<Interface>, std::string>::err(
                "Factory function '" + factory_symbol + "' in '" + so_path +
                "' threw: " + e.what());
        } catch (...) {
            ::dlclose(dl_handle);
            return Result<PluginHandle<Interface>, std::string>::err(
                "Factory function '" + factory_symbol + "' in '" + so_path +
                "' threw unknown exception");
        }
        if (!instance) {
            ::dlclose(dl_handle);
            return Result<PluginHandle<Interface>, std::string>::err(
                "Factory function '" + factory_symbol + "' in '" + so_path + "' returned null");
        }

        DRONE_LOG_INFO("[PluginLoader] Successfully loaded plugin from '{}'", so_path);

        return Result<PluginHandle<Interface>, std::string>::ok(
            PluginHandle<Interface>(dl_handle, instance));
    }

    // Static-only class.
    PluginLoader()  = delete;
    ~PluginLoader() = delete;
};

}  // namespace drone::util

#endif  // HAVE_PLUGINS
