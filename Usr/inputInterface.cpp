#include "inputInterface.h"

static HHOOK  hhkMouse = NULL; //鼠标钩子句柄;
static HINSTANCE g_hInstDll = NULL; //本dll的实例句柄;
static HWND g_hWnd = NULL;//调用dll的主窗口句柄
mouseHook_t mouse;
keyboardHook_t keyboard;
HWND m_hWnd; //全局窗口

BOOL WINAPI StartHookMouse(HWND hWnd);
VOID WINAPI StopHookMouse();

LRESULT CALLBACK WindowProc(_In_  HWND hwnd, _In_  UINT uMsg, _In_  WPARAM wParam, _In_  LPARAM lParam)
{
	switch (uMsg)
	{
	case WM_COMMAND:
		break;
	case WM_PAINT:
		break;
	case WM_DESTROY:
		PostQuitMessage(0);
		break;
	case WM_INPUT:
	{
		UINT dwSize = 0;
		GetRawInputData((HRAWINPUT)lParam, (UINT)RID_INPUT, NULL, &dwSize, sizeof(RAWINPUTHEADER));//取数据,第一次调用函数将获取需要的字节大小		
		LPBYTE lpbBuffer = new BYTE[dwSize];//分配指定的缓冲区大小
		GetRawInputData((HRAWINPUT)lParam, (UINT)RID_INPUT, (LPVOID)lpbBuffer, (PUINT)&dwSize, (UINT)sizeof(RAWINPUTHEADER));//第二次调用获取原始输入数据，读入lpbBuffer

		RAWINPUT* raw = (RAWINPUT*)lpbBuffer;
		if (raw->header.dwType == RIM_TYPEKEYBOARD)//这里可以截获所有键盘信息,如需区分不同的键盘输入信息,可以通过设备句柄判断。
		{
			//读取键盘名称
			UINT bufferSize;//为键盘设备名准备缓冲区大小
			GetRawInputDeviceInfo(raw->header.hDevice, RIDI_DEVICENAME, NULL, &bufferSize);
			WCHAR* RawDevName = new WCHAR[bufferSize];
			GetRawInputDeviceInfo(raw->header.hDevice, RIDI_DEVICENAME, RawDevName, &bufferSize);//将设备名读入缓冲区RawDevName
			//printf("dev=%ls\n", RawDevName);
			delete[] RawDevName;

			//获取键盘输入
			char keytext[10] = { 0 };
			BYTE state[256] = { 0 };
			ToAscii(raw->data.keyboard.VKey, raw->data.keyboard.MakeCode, state, (LPWORD)keytext, 0);//通过虚拟键盘码得到名字

			BYTE KeyIndex = 16;
			switch (raw->data.keyboard.VKey)
			{
			case _W_KV:
				KeyIndex = 0;
				break;
			case _S_KV:
				KeyIndex = 1;
				break;
			case _A_KV:
				KeyIndex = 2;
				break;
			case _D_KV:
				KeyIndex = 3;
				break;
			case _SHIFT_KV:
				KeyIndex = 4;
				break;
			case _CTRL_KV:
				KeyIndex = 5;
				break;
			case _Q_KV:
				KeyIndex = 6;
				break;
			case _E_KV:
				KeyIndex = 7;
				break;
			case _R_KV:
				KeyIndex = 8;
				break;
			case _F_KV:
				KeyIndex = 9;
				break;
			case _G_KV:
				KeyIndex = 10;
				break;
			case _Z_KV:
				KeyIndex = 11;
				break;
			case _X_KV:
				KeyIndex = 12;
				break;
			case _C_KV:
				KeyIndex = 13;
				break;
			case _V_KV:
				KeyIndex = 14;
				break;
			case _B_KV:
				KeyIndex = 15;
				break;
			case _ESC_KV:
				KeyIndex = 18;
				break;
			case _O_KV:
				KeyIndex = 19;
				break;
			case _P_KV:
				KeyIndex = 20;
				break;
			case _F7_KV:
				KeyIndex = 21;
				break;
			}

			if (raw->data.keyboard.Message == WM_KEYDOWN)
			{
				if (KeyIndex < 32)
				{
					keyboard.setKeyboardMask(KeyIndex);
				}
			}
			else if (raw->data.keyboard.Message == WM_KEYUP)
			{
				keyboard.resetKeyboardMask(KeyIndex);
			}
		}
		delete[] lpbBuffer;
		return 0;
	}
	default:
		return DefWindowProc(hwnd, uMsg, wParam, lParam);
	}
	return 0;
}

void keyboardHook_t::RegKeyboardRawInput(HWND hWnd)
{
	RAWINPUTDEVICE rawInputDevice[1];
	rawInputDevice[0].usUsagePage = 0x01;//设备类
	rawInputDevice[0].usUsage = 0x06;//设备类内的具体设备
	rawInputDevice[0].dwFlags = RIDEV_INPUTSINK;//意味着即使窗口失去焦点位置，仍然会一直接收输入消息
	rawInputDevice[0].hwndTarget = hWnd;
	if (RegisterRawInputDevices(rawInputDevice, 1, sizeof(rawInputDevice[0])) == FALSE)
	{
		printf_s(" - RegisterRawInputDevices failed\r\n");
	}
}

int keyboardHook_t::init()
{
	/* 创建窗口 */
	HINSTANCE hInst;

	hInst = GetModuleHandle(NULL);	//获取一个应用程序或动态链接库的模块句柄	
	WNDCLASSEX wcx;
	wcx.cbSize = sizeof(wcx);
	wcx.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC | CS_NOCLOSE;//窗口的风格	
	wcx.lpfnWndProc = WindowProc;     //采用自定义消息处理函数
	wcx.cbClsExtra = 0;                // no extra class memory  
	wcx.cbWndExtra = 0;                // no extra window memory  
	wcx.hInstance = hInst;         //当前应用程序的实例句柄 
	wcx.hIcon = LoadIcon(NULL, IDI_APPLICATION); //图标风格		
	wcx.hCursor = LoadCursor(NULL, IDC_ARROW);  //鼠标风格
	wcx.hbrBackground = (HBRUSH)WHITE_BRUSH; //背景色
	wcx.lpszMenuName = NULL;  //菜单名	
	wcx.lpszClassName = _T("ITSMYOSDCLASS"); //窗口类的名称
	wcx.hIconSm = NULL;

	if (!RegisterClassEx(&wcx))
	{
		printf_s(" - Register classEx failed!\r\n");
		return -1;
	}
	else printf_s(" - Register classEx succeed!\r\n");

	//窗口在屏幕上的显示位置
	int OSDleft = GetSystemMetrics(SM_CXSCREEN) / 2 - 300;
	int OSDTop = GetSystemMetrics(SM_CYSCREEN) / 2;

	m_hWnd = CreateWindowEx(
		WS_EX_LAYERED | WS_EX_TOOLWINDOW | WS_EX_TOPMOST | WS_EX_TRANSPARENT | WS_EX_NOACTIVATE,//窗口的扩展风格
		wcx.lpszClassName,//上面注册的类名lpszClassName，要完全一致  
		NULL,
		WS_VISIBLE | WS_POPUP,//窗口的风格
		OSDleft,//窗口相对于父级的X坐标  
		OSDTop,//窗口相对于父级的Y坐标 
		300,//窗口的宽度  
		300,//窗口的高度 
		(HWND)NULL,//没有父窗口，为(HWND)NULL//GetDesktopWindow(),
		(HMENU)NULL,//没有菜单，为NULL  
		hInst,//当前应用程序的实例句柄  
		(LPVOID)NULL);	//没有附加数据，为NULL  

	if (!m_hWnd)
	{
		printf_s(" - Create window failed!\r\n");
		return -1;
	}
	else printf_s(" - Create window succeed!\r\n");

	RegKeyboardRawInput(m_hWnd);

}

void keyboardHook_t::setKeyboardMask(INT index)
{
	if (index < 32 && index >= 0)
	{
		this->keyPressedMask |= (1 << index);
		this->keyTriggeredMask |= (1 << index);
	}
}

void keyboardHook_t::resetKeyboardMask(INT index)
{
	if (index < 32 && index >= 0)
	{
		this->keyPressedMask &= ~(1 << index);
	}
}

BYTE keyboardHook_t::isKeyPressed(BYTE key)
{
	if (this->keyPressedMask & (1 << key))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

BYTE keyboardHook_t::isKeyTriggered(BYTE key)
{
	if (this->keyTriggeredMask & (1 << key))
	{
		this->keyTriggeredMask &= ~(1 << key);
		return 1;
	}
	else
	{
		return 0;
	}
}

mouseHook_t::mouseHook_t()
{

}

mouseHook_t::~mouseHook_t()
{
	StopHookMouse();
}

void mouseHook_t::init()
{
	StartHookMouse(m_hWnd);
	ClipCursor(NULL);
}

void mouseHook_t::updateMousePos(CPoint pos)
{
	this->lastPos = this->mousePos;
	this->mousePos = pos;
	this->timeWithoutUpdate = 0;
}

void mouseHook_t::setMousePressed(BYTE lr)
{
	this->mousePressMask |= (1 << (lr - 16));
}

void mouseHook_t::resetMousePressed(BYTE lr)
{
	this->mousePressMask &= ~(1 << (lr - 16));
}

bool mouseHook_t::ifMousePressed(BYTE lr)
{
	if (this->mousePressMask & (1 << (lr - 16)))
		return true;
	else
		return false;
}

void mouseHook_t::setExitFlag(bool exflag)
{
	this->exitFlag = exflag;
}

/* 解算鼠标的速度 */
void mouseHook_t::resolveVelocity(float timegap)
{
	this->velocity.vx = (this->mousePos.x - this->lastPos.x) / timegap;
	this->velocity.vy = (this->mousePos.y - this->lastPos.y) / timegap;
	if (mousePos.x < 800 || mousePos.x>1120 || mousePos.y < 400 || mousePos.y>680)
	{
		if (!exitFlag)
		{
			mousePos.x = 1920 / 2;
			mousePos.y = 1080 / 2;
			lastPos = mousePos;
			SetCursorPos(mousePos.x, mousePos.y);
			//printf_s("%d,%d\r\n", mousePos.x, mousePos.y);
		}
	}
	timeWithoutUpdate++;
	if (timeWithoutUpdate > 10)
	{
		velocity.vx = velocity.vy = 0;
	}
}

//低级鼠标钩子过程
LRESULT CALLBACK LowLevelMouseProc(
	int nCode,      // hook code
	WPARAM wParam,  // message identifier
	LPARAM lParam   // mouse coordinates
)
{
	CPoint  _mousepoint;
	//有鼠标消息时，将其发给主程序
	if (g_hWnd != NULL && nCode == HC_ACTION)
	{
		GetCursorPos(&_mousepoint);
		mouse.updateMousePos(_mousepoint);
		switch (wParam)
		{
		case WM_LBUTTONDOWN:
			mouse.setMousePressed(_MOUSE_L);
			break;
		case WM_LBUTTONUP:
			mouse.resetMousePressed(_MOUSE_L);
			break;
		case WM_RBUTTONDOWN:
			mouse.setMousePressed(_MOUSE_R);
			break;
		case WM_RBUTTONUP:
			mouse.resetMousePressed(_MOUSE_R);
			break;
		default:
			break;
		}
	}
	return CallNextHookEx(hhkMouse, nCode, wParam, lParam);
}

//安装低级鼠标钩子，从而截获系统所有的鼠标消息
BOOL WINAPI StartHookMouse(HWND hWnd)
{
	g_hWnd = hWnd;
	hhkMouse = ::SetWindowsHookEx(WH_MOUSE_LL, LowLevelMouseProc, g_hInstDll, 0);
	if (hhkMouse == NULL)
	{
		printf_s(" - Start mouse hook failed!\r\n");
		return FALSE;
	}
	else
	{
		printf_s(" - Start mouse hook successfully!\r\n");
		return TRUE;
	}
}

//卸载低级鼠标钩子
VOID WINAPI StopHookMouse()
{
	if (hhkMouse != NULL)
	{
		::UnhookWindowsHookEx(hhkMouse);
	}
}
