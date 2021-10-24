#include <locale>
#include <string>

#define SFML_API_EXPORT
#define SFML_API_IMPORT
#define SFML_SYSTEM_API
#define SFML_WINDOW_API SFML_API_EXPORT

namespace sf
{
namespace priv
{
    class GlContext;
    class WindowImpl;
}

class SFML_WINDOW_API GlResource
{
 public:
	GlResource();
};

class SFML_SYSTEM_API NonCopyable
{
 public:
 NonCopyable();
};

class SFML_SYSTEM_API String
{
public:
String();
String(const char* ansiString, const std::locale& locale = std::locale());
String(char ansiChar, const std::locale& locale = std::locale());
};
	
class Event;
typedef unsigned int Uint32;
struct ContextSettings {};
enum Style { Default };
	
class SFML_WINDOW_API VideoMode
{
public:
    VideoMode();
    VideoMode(unsigned int modeWidth, unsigned int modeHeight, unsigned int modeBitsPerPixel = 32);
};
	
class SFML_WINDOW_API Window : GlResource, NonCopyable
{
public:
    //Window();
	// used
	Window(VideoMode mode, const String& title, Uint32 style = Style::Default, const ContextSettings& settings = ContextSettings());
	
    //explicit Window(WindowHandle handle, const ContextSettings& settings = ContextSettings());
	// used
    virtual ~Window();
    //void create(VideoMode mode, const String& title, Uint32 style = Style::Default, const ContextSettings& settings = ContextSettings());
    //void create(WindowHandle handle, const ContextSettings& settings = ContextSettings());
	// used
    void close();
	// used
	bool isOpen() const;
    //const ContextSettings& getSettings() const;
	// used
    bool pollEvent(Event& event);
    //bool waitEvent(Event& event);
    //Vector2i getPosition() const;
    //void setPosition(const Vector2i& position);
    //Vector2u getSize() const;
    //void setSize(const Vector2u& size);
    //void setTitle(const String& title);
    //void setIcon(unsigned int width, unsigned int height, const Uint8* pixels);
    //void setVisible(bool visible);
    //void setVerticalSyncEnabled(bool enabled);
    //void setMouseCursorVisible(bool visible);
    //void setMouseCursorGrabbed(bool grabbed);
    //void setMouseCursor(const Cursor& cursor);
    //void setKeyRepeatEnabled(bool enabled);
	// used
	void setFramerateLimit(unsigned int limit);
	//void setJoystickThreshold(float threshold);
    //bool setActive(bool active = true) const;
    //void requestFocus();
    //bool hasFocus() const;
	// used
    void display();
    //WindowHandle getSystemHandle() const;
protected:
    //virtual void onCreate();
    //virtual void onResize();

private:
    //bool filterEvent(const Event& event);
    //void initialize();
};

} // namespace sf

