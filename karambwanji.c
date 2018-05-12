#define CONTROL_MBOX mailbox1
#define STOP_MBOX mailbox2

// XXX idk how long the message buffer is actually
// bluetooth message max is 58 bytes?
#define MESSAGE_BUF_SIZE 64
#define MESSAGE_WAIT_DELAY 100

#define FW_NAME "karambwanji 0.73"

#define OFF_FIRMWARE_NAME 0
#define OFF_POS_U         20
#define OFF_POS_V         24
#define OFF_POS_X         28
#define OFF_POS_Y         32
#define OFF_IP            36

#define DEFAULT_SPEED 50
#define VSPEED_MIN 5
#define USPEED_MIN 10
//#define VSPEED_MAX DEFAULT_SPEED
//#define USPEED_MAX DEFAULT_SPEED
#define MOVE_RESOLUTION 5
#define MOVE_EPSILON 2
#define TURN_DELAY_V 250
//#define MOVE_DELAY 250

//#define DEBUG_MATH
#define DEBUG_MOVE
//#define DEBUG_MOVE_POS

#define PANIC_ON_ERROR

#define U2MM(u) (((float)(u))/100.0)
#define MM2U(m) ((long)(((m)*100.0)))

// some mentally challenged robotc developer though
// radiansToDegrees should return short and not float
#define RAD2DEG(rad) ((rad)*360.0/(2.0*PI))
#define DEG2RAD(deg) ((deg)*2.0*PI/360.0)

// XXX: This guy https://github.com/cavenel/ev3-print3rbot/blob/master/writer.py
// uses 2970, but my tests gave 3635
#define A2TC(theta) ((theta)*3635.0/90.0)
#define TC2A(tacho) ((tacho)*90.0/3635.0)

// Device parameters (mm)
// Note: if you change HW_R1 you must change HW_Ay and vice versa
// such that uv_to_xy maps origin to origin
#define HW_D  24.0
#define HW_R1 145.0
#define HW_R2 89.0
#define HW_Ax (-HW_D/2)
#define HW_Ay (-104.0)
#define HW_Bx (-HW_Ax)
#define HW_By HW_Ay

void init();
void reset_position();
void reset_axes();
void reset_run_state();

void main_loop();

unsigned short wait_mes(TMailboxIDs mbox);
void loop_watch_mbox(TMailboxIDs mbox);
void dispatch_buf(ubyte *buf, unsigned short sz);
unsigned short dispatch_cmd(ubyte *buf);

void cc_intersect(float *ix1, float *iy1, float *ix2, float *iy2,
	float x1, float y1, float r1, float x2, float y2, float r2);
void xy_to_uv(long *u, long *v, long x, long y);
void uv_to_xy(long *x, long *y, long u, long v);

void move_xy(long x, long y, float speed);
void move_uv(long b, long c, float speed);
void move_delta_xy(long dx, long dy, float speed);
void move_delta_uv(long db, long dc, float speed);
void move_z(int up);

void set_ip(unsigned int ip);
void write(ubyte *data, unsigned short off, unsigned short len);

void panic();

void test_circle(long r, int N);

task update_state();
task poll_control();

typedef struct {
	int speed;
} run_opt_s;

run_opt_s run_opt;

typedef struct {
	int ld_v;
	unsigned int ip;
} run_state_s;

run_state_s run_state;

task main() {
	writeDebugStream("--- INIT ---\n");
	nxtDisplayTextLine(0, "Init");

	init();

	reset_axes();
	reset_position();
	reset_run_state();

	StartTask(update_state);
	StartTask(poll_control);

	main_loop();
}

task update_state(){
	long buf[4];
	for(;;){
		buf[0] = nMotorEncoder[motorB];
		buf[1] = nMotorEncoder[motorC];
		uv_to_xy(&buf[2], &buf[3], buf[0], buf[1]);
		write((ubyte *) buf, OFF_POS_U, 16);
		wait1Msec(100);
	}
}

task poll_control(){
	nxtDisplayTextLine(0, "P:Active");
	loop_watch_mbox(CONTROL_MBOX);
}

void init(){
	bFloatDuringInactiveMotorPWM = false;
	for(tMotor m=motorA;m<=motorC;m++){
		nMotorPIDSpeedCtrl[m] = mtrNoReg;
	}
	SensorType[S1] = sensorTouch;
	SensorType[S2] = sensorTouch;

	run_opt.speed = DEFAULT_SPEED;

	ubyte fwname[20];
	strcpy((char *)fwname, FW_NAME);
	write(fwname, OFF_FIRMWARE_NAME, 20);
}

void reset_position(){
	for(tMotor m=motorA;m<=motorC;m++){
		nMotorEncoder[m] = 0;
	}
}

void reset_axes(){
	motor[motorB] = -100;
	motor[motorC] = 100;
	for(;;){
		if(SensorValue[S1])
			motor[motorB] = 0;
		if(SensorValue[S2])
			motor[motorC] = 0;
		if(SensorValue[S1] && SensorValue[S2])
			break;
	}
}

void reset_run_state(){
	//run_state.ld_u = 0;
	run_state.ld_v = 0;
	run_state.ip = 0;
}

void main_loop(){
	nxtDisplayTextLine(3, "Running");
	for(;;){
		unsigned short sz = wait_mes(STOP_MBOX);

#ifdef PANIC_ON_ERROR
		if(sz > 1){
			nxtDisplayTextLine(0, "Invalid stop msg");
			nxtDisplayTextLine(1, "Expected 1 byte");
			nxtDisplayTextLine(2, "Got %d", sz);
			panic();
		}
#endif

		ubyte cmd;
		cCmdMessageRead(&cmd, 1, STOP_MBOX);
		switch(cmd){
			case 'S':
				nxtDisplayTextLine(3, "Paused");
				StopTask(poll_control);
				cCmdBTPurgeRcvBuffer();
				break;
			case 'R':
				nxtDisplayTextLine(3, "Resumed");
				StartTask(poll_control);
				break;
#ifdef PANIC_ON_ERROR
			default:
				nxtDisplayTextLine(0, "Invalid stop cmd");
				nxtDisplayTextLine(1, "Cmd: '%c' (%d)", cmd, cmd);
				panic();
				break;
#endif
		}
	}
}

unsigned short wait_mes(TMailboxIDs mbox){
	for(;;){
		short s = cCmdMessageGetSize(mbox);
		if(s)
			return s;
		wait1Msec(MESSAGE_WAIT_DELAY);
	}

	return -1;
}

void loop_watch_mbox(TMailboxIDs mbox){
	ubyte buf[MESSAGE_BUF_SIZE];
	for(;;){
		nxtDisplayTextLine(6, "Idle");
		unsigned short sz = wait_mes(mbox);
		nxtDisplayTextLine(6, "R%d/%d", sz, MESSAGE_BUF_SIZE);

		if(sz > MESSAGE_BUF_SIZE){
#ifdef PANIC_ON_ERROR
			nxtDisplayTextLine(0, "Recv buffer overflow");
			nxtDisplayTextLine(1, "Read: %d", sz);
			nxtDisplayTextLine(2, "Size: %d", MESSAGE_BUF_SIZE);
			panic();
#else
			sz = MESSAGE_BUF_SIZE;
#endif
		}

		cCmdMessageRead(buf, sz, mbox);
		dispatch_buf(buf, sz);
	}
}

void dispatch_buf(ubyte *buf, unsigned short sz){
	unsigned short s = 0;
	while(s < sz){
		nxtDisplayTextLine(5, "B@%d/%d", s, sz);
		nxtDisplayTextLine(6, "%02x %02x %02x %02x", buf[0], buf[1], buf[2], buf[3]);
		//wait1Msec(5000);
		unsigned short cs = dispatch_cmd(buf);
		s += cs;
		buf += cs;
	}

#ifdef PANIC_ON_ERROR
	if(s > sz){
		nxtDisplayTextLine(0, "Buffer overflow");
		nxtDisplayTextLine(1, "Buf: %d", (long)buf);
		nxtDisplayTextLine(2, "Size: %d", sz);
		nxtDisplayTextLine(3, "Read: %d", s);
		panic();
	}
#endif
}

unsigned short dispatch_cmd(ubyte *buf){
	set_ip(run_state.ip+1);

	ubyte opcode = *buf;
	writeDebugStream("dispatch_cmd: opcode: %d\n", opcode);
	switch(opcode){
		case 'M':
			move_xy(*((long *)&buf[1]),*((long *)&buf[5]), run_opt.speed);
#ifdef MOVE_DELAY
			wait1Msec(MOVE_DELAY);
#endif
			return 9;
		case 'm':
			move_delta_xy(*((long *)&buf[1]), *((long *)&buf[5]), run_opt.speed);
#ifdef MOVE_DELAY
			wait1Msec(MOVE_DELAY);
#endif
			return 9;
		case 'Z':
			move_z(buf[1]);
			return 2;
		case 'I':
			set_ip(buf[1]|buf[2]<<8);
			return 3;
		case 'R':
			reset_axes();
			return 1;
		case 'z':
			reset_position();
			return 1;
	}

#ifdef PANIC_ON_ERROR
	nxtDisplayTextLine(0, "Invalid opcode");
	nxtDisplayTextLine(1, "OP: '%c' (%d)", opcode, opcode);
	nxtDisplayTextLine(2, "Buf: %d", (long) buf);
	panic();
	return 0;
#else
	writeDebugStream("Invalid opcode\n");
	// Skip the whole buffer
	return (unsigned short) (-1);
#endif
}

int min(int a, int b){
	return a < b ? a : b;
}

int max(int a, int b){
	return a > b ? a : b;
}

int clamp(int x, int m, int M){
	return x < m ? m : (x > M ? M : x);
}

// Compute intersects (ix1, iy1), (ix2, iy2)
// of circles at (x1, y1) of radius r1 and (x2, y2) of radius r2
// algorithm: https://math.stackexchange.com/a/1367732
void cc_intersect(float *ix1, float *iy1, float *ix2, float *iy2,
	float x1, float y1, float r1, float x2, float y2, float r2){

	float dx = x1 - x2;
	float dy = y1 - y2;
	float R2 = dx*dx + dy*dy;

	float rr2 = r1*r1 - r2*r2;
	float a = rr2 / R2;
	float b = sqrt(2.0*(r1*r1 + r2*r2)/R2 - a*a - 1.0);

#ifdef DEBUG_MATH
	writeDebugStream("cc_int: a=%f b=%f, R^2=%f, rr2=%f\n", a, b, R2, rr2);
#endif

	*ix1 = ( (x1+x2) + a*(x2-x1) + b*(y2-y1) ) / 2.0;
	*iy1 = ( (y1+y2) + a*(y2-y1) + b*(x1-x2) ) / 2.0;

	*ix2 = ( (x1+x2) + a*(x2-x1) - b*(y2-y1) ) / 2.0;
	*iy2 = ( (y1+y2) + a*(y2-y1) - b*(x1-x2) ) / 2.0;

#ifdef DEBUG_MATH
	writeDebugStream("cc_int: (%f, %f)r%f + (%f, %f)r%f\n", x1, y1, r1, x2, y2, r2, );
	writeDebugStream("     -> (%f, %f) & (%f, %f)\n", *ix1, *iy1, *ix2, *iy2);
#endif
}

// Pretty much the same transform as in here
// https://github.com/cavenel/ev3-print3rbot/blob/master/writer.py
void xy_to_uv(long *u, long *v, long x, long y){
	const float Ex = U2MM(x), Ey = U2MM(y);
	float Cx, Cy, Dx, Dy;

	float ix, iy;

	cc_intersect(&Cx, &Cy, &ix, &iy, HW_Ax, HW_Ay, HW_R2, Ex, Ey, HW_R1);
	if(Cx > ix){
		Cx = ix;
		Cy = iy;
	}

	cc_intersect(&Dx, &Dy, &ix, &iy, HW_Bx, HW_By, HW_R2, Ex, Ey, HW_R1);
	if(Dx < ix){
		Dx = ix;
		Dy = iy;
	}

#ifdef DEBUG_MATH
	writeDebugStream("uv_to_xy: C(%f, %f) D(%f, %f)\n", Cx, Cy, Dx, Dy);
#endif

#ifdef PANIC_ON_ERROR
	if(Cx > HW_Ax || Dx < HW_Bx){
		nxtDisplayTextLine(0, "Invalid coords");
		nxtDisplayTextLine(1, "xy (%f, %f)", Ex, Ey);
		nxtDisplayTextLine(2, "A(%f, %f)", HW_Ax, HW_Ay);
		nxtDisplayTextLine(3, "B(%f, %f)", HW_Bx, HW_By);
		nxtDisplayTextLine(4, "C(%f, %f)", Cx, Cy);
		nxtDisplayTextLine(5, "D(%f, %f)", Dx, Dy);
		panic();
	}
#endif

	float alpha = acos((HW_Ax-Cx)/HW_R2);
	float beta = acos((Dx-HW_Bx)/HW_R2);

#ifdef DEBUG_MATH
	writeDebugStream("uv_to_xy: alpha(%f) = acos(%f), beta(%f) = acos(%f)\n",
		alpha, (HW_Ax-Cx)/HW_R2, (Dx-HW_Bx)/HW_R2, beta );
#endif

	*u = (long) (A2TC(RAD2DEG(alpha)));
	*v = (long) -(A2TC(RAD2DEG(beta)));
}

void uv_to_xy(long *x, long *y, long u, long v){
	float alpha = TC2A(DEG2RAD(u));
	float beta = TC2A(DEG2RAD(u));

	float Cx = HW_Ax - HW_R2 * cos(alpha);
	float Cy = HW_Ay + HW_R2 * sin(alpha);
	float Dx = HW_Bx + HW_R2 * cos(beta);
	float Dy = HW_By + HW_R2 * sin(beta);

	float ix1, iy1, ix2, iy2;
	cc_intersect(&ix1, &iy1, &ix2, &iy2, Cx, Cy, HW_R1, Dx, Dy, HW_R1);
	if(iy2 > iy1){
		ix1 = ix2;
		iy1 = iy2;
	}

	*x = MM2U(ix1);
	*y = MM2U(iy1);
}

void move_xy(long x, long y, float speed){
	long u, v;
	xy_to_uv(&u, &v, x, y);
	writeDebugStream("xy(%d, %d) => uv(%d, %d)\n", x, y, u, v);
	move_uv(u, v, speed);
}

void move_uv(long b, long c, float speed){
#ifdef DEBUG_MOVE
	writeDebugStream("move_enc_uv (%d, %d) => (%d, %d)\n",
		nMotorEncoder[motorB], nMotorEncoder[motorC], b, c);
#ifdef DEBUG_MOVE_POS
	writeDebugStream("du    dv    u0    v0    D     uadj  vadj\n");
#endif
#endif

	int mu = sgn(b - nMotorEncoder[motorB]);
	int mv = sgn(c - nMotorEncoder[motorC]);

	if(mv != run_state.ld_v){
		// pen seems to lag behind when changing
		// directions so fix that
		motor[motorC] = mv * 10;
		run_state.ld_v = mv;
		wait1Msec(TURN_DELAY_V);
	}

	for(;;){
		long du = max(0, mu * (b - nMotorEncoder[motorB]));
		long dv = max(0, mv * (c - nMotorEncoder[motorC]));
		long d2 = du*du + dv*dv;
		if(d2 <= MOVE_EPSILON)
			break;

		float f = sqrt(d2);
		int pu = (int) (speed * du / f);
		int pv = (int) (speed * dv / f);

#if defined(DEBUG_MOVE) && defined(DEBUG_MOVE_POS)
		writeDebugStream("%-5d %-5d %-5d %-5d %-5.2f ", du, dv, pu, pv, f);
#endif

		if(pu > 0 && pu < USPEED_MIN){
			pv = (int) ( ((float)pv) * ((float)USPEED_MIN)/((float)pu) );
			pu = USPEED_MIN;
		}

		if(pv > 0 && pv < VSPEED_MIN){
			pu = (int) ( ((float)pu) * ((float)VSPEED_MIN)/((float)pv) );
			pv = VSPEED_MIN;
		}

#ifdef USPEED_MAX
		pu = mu * min(pu, USPEED_MAX);
#else
		pu = mu * min(pu, speed);
#endif

#ifdef VSPEED_MAX
		pv = mv * min(pv, VSPEED_MAX);
#else
		pv = mv * min(pv, speed);
#endif

#if defined(DEBUG_MOVE) && defined(DEBUG_MOVE_POS)
		writeDebugStream("%-5d %-5d\n", pu, pv);
#endif

		motor[motorB] = pu;
		motor[motorC] = pv;

		wait1Msec(MOVE_RESOLUTION);
	}

	motor[motorB] = 0;
	motor[motorC] = 0;

#ifdef DEBUG_MOVE
	writeDebugStream("move_enc_uv: finished @ (%d, %d) D(%d, %d)\n",
		nMotorEncoder[motorB], nMotorEncoder[motorC],
		b - nMotorEncoder[motorB], c - nMotorEncoder[motorC]
	);
#endif
}

void move_delta_uv(long db, long dc, float speed){
	move_uv(nMotorEncoder[motorB] + db, nMotorEncoder[motorC] + dc, speed);
}

void move_delta_xy(long dx, long dy, float speed){
	long x, y;
	uv_to_xy(&x, &y, nMotorEncoder[motorB], nMotorEncoder[motorC]);
	move_xy(x+dx, y+dy, speed);
}

void move_z(int up){
	motor[motorA] = up ? 50 : -50;
	for(;;){
		long enc = nMotorEncoder[motorA];
		wait1Msec(50);
		if(enc == nMotorEncoder[motorA])
			break;
	}
	motor[motorA] = 0;
}

void set_ip(unsigned int ip){
	run_state.ip = ip;
	write((byte *) &ip, OFF_IP, 2);
	nxtDisplayTextLine(7, "IP:%d", ip);
}

void write(ubyte *data, unsigned short off, unsigned short len){
	int res;
	nxtWriteIOMap("Comm.mod", res, data, 1289+off, len);
}

void panic(){
	PlaySound(soundBeepBeep);
	for(;;) wait1Msec(1000);
}

void test_circle(long r, int N){
	// make calculations in mm to avoid overflows
	float r_mm = U2MM(r);
	for(int i=0;i<N;i++){
		float x_mm = (float) (r_mm*cos(-2*PI*((float)i)/((float)N)));
		float y_mm = (float) ( (r_mm+5) + r_mm*sin(-2*PI*((float)i)/((float)N)) );
		move_xy(MM2U(x_mm), MM2U(y_mm), run_opt.speed);
		wait1Msec(100);
	}
}
