#pragma once
#define _AFXDLL
#include <afxwin.h>         // MFC 核心组件和标准组件
#include <afxext.h>         // MFC 扩展

#define _W          0
#define _S          1
#define _A          2
#define _D          3
#define _SHIFT      4
#define _CTRL       5
#define _Q          6
#define _E          7
#define _R          8
#define _F          9
#define _G          10
#define _Z          11
#define _X          12
#define _C          13
#define _V          14
#define _B          15
#define _MOUSE_L	16
#define _MOUSE_R	17
#define _ESC		18
/* below are extended */
#define _O			19
#define _P			20
#define _F7			21

#define _W_KV		0x57
#define _S_KV		0x53
#define _A_KV		0x41
#define _D_KV		0x44
#define _SHIFT_KV	0x10
#define _CTRL_KV	0x11
#define _Q_KV		0x51
#define _E_KV		0x45
#define _R_KV		0x52
#define _F_KV		0x46
#define _G_KV		0x47
#define _Z_KV		0x5a
#define _X_KV		0x58
#define _C_KV		0x43
#define _V_KV		0x56
#define _B_KV		0x42
#define _ESC_KV		0x1b
#define _O_KV		0x49
#define _P_KV		0x50
#define _F7_KV		0x76

class keyboardHook_t
{
public:
	keyboardHook_t() {};
	int init();
	void setKeyboardMask(INT index);
	void resetKeyboardMask(INT index);
	BYTE isKeyPressed(BYTE key);
	BYTE isKeyTriggered(BYTE key);
private:
	UINT32 keyPressedMask;
	UINT32 keyTriggeredMask;
	void RegKeyboardRawInput(HWND hWnd);
};

extern keyboardHook_t keyboard;

struct mouseVelocity_t
{
	float vx;
	float vy;
};

class mouseHook_t
{
public:
	mouseHook_t();
	~mouseHook_t();
	void init();
	void updateMousePos(CPoint pos);
	void setMousePressed(BYTE lr);
	void resetMousePressed(BYTE lr);
	void resolveVelocity(float timegap);
	bool ifMousePressed(BYTE lr);
	void setExitFlag(bool exflag);
	mouseVelocity_t velocity;
private:
	unsigned long timeWithoutUpdate;
	CPoint mousePos;
	CPoint lastPos;
	BYTE mousePressMask;
	bool exitFlag;
};

extern mouseHook_t mouse;
