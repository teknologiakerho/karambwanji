#define CONTROL_MBOX mailbox1
#define STOP_MBOX mailbox2

// XXX idk how long the message buffer is actually
// bluetooth message max is 58 bytes?
#define MESSAGE_BUF_SIZE 64
#define MESSAGE_WAIT_DELAY 100

#define FW_NAME "karambwanji 0.73"

#define OFF_FIRMWARE_NAME 0
#define OFF_POS_U         20
#define OFF_POS_V         22
#define OFF_POS_X         24
#define OFF_POS_Y         26
#define OFF_IP            28

#define DEFAULT_SPEED 15
#define VSPEED_MIN 5
#define USPEED_MIN 10
#define VSPEED_MAX 20
#define USPEED_MAX 20
#define MOVE_RESOLUTION 5
#define MOVE_EPSILON 2
#define TURN_DELAY_V 250
//#define MOVE_DELAY 250

#define DEBUG_UV
#define DEBUG_MOVE
#define DEBUG_MOVE_POS

#define PANIC_ON_ERROR

// Device parameters
#define L1 70
#define L2 110
#define R 15
#define KU 1.0
#define KV (90.0/16.0)
#define U0 0
#define V0 94.1
#define X_OFFSET 0
#define Y_OFFSET 80

void init();
void reset_position();
void reset_axes();
void reset_run_state();

void main_loop();

unsigned short wait_mes(TMailboxIDs mbox);
void loop_watch_mbox(TMailboxIDs mbox);
void dispatch_buf(ubyte *buf, unsigned short sz);
unsigned short dispatch_cmd(ubyte *buf);

int min(int a, int b);
int max(int a, int b);
int clamp(int x, int m, int M);
void xy_to_uv(int *u, int *v, int x, int y);
void uv_to_xy(int *x, int *y, int u, int v);

void move_enc_xy(long x, long y, float speed);
void move_enc_uv(long b, long c, float speed);
void move_delta_xy(long dx, long dy, float speed);
void move_delta_uv(long db, long dc, float speed);
void move_z(int up);

void set_ip(unsigned int ip);
void write(ubyte *data, unsigned short off, unsigned short len);

void panic();

void test_circle(int r, int N);

task update_state();
task poll_control();

typedef struct {
	int speed;
} run_opt_s;

run_opt_s run_opt;

typedef struct {
	int ld_v;
	unsigned int ip;
	int pen_up;
} run_state_s;

run_state_s run_state;

task main() {
	writeDebugStream("--- INIT ---\n");
	nxtDisplayTextLine(0, "Init");

	init();

	//reset_axes();
	reset_position();
	reset_run_state();

	StartTask(update_state);
	StartTask(poll_control);

	main_loop();
}

task update_state(){
	ubyte buf[8];
	for(;;){
		*((int *)buf[0]) = nMotorEncoder[motorB];
		*((int *)buf[2]) = nMotorEncoder[motorC];
		uv_to_xy((int *) &buf[4], (int *) &buf[6], *((int *)buf[0]), *((int *)buf[2]));
		write(buf, OFF_POS_U, 8);
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

	run_opt.speed = DEFAULT_SPEED;

	ubyte fwname[20];
	strcpy((char *)fwname, FW_NAME);
	write(fwname, OFF_FIRMWARE_NAME, 20);

	// make sure pen is up after init
	run_state.pen_up = 1;
	move_z(0);
	move_z(1);
}

void reset_position(){
	for(tMotor m=motorA;m<=motorC;m++){
		nMotorEncoder[m] = 0;
	}
}

void reset_axes(){
	// no need to touch x here
	motor[motorC] = -20;
	for(;;){
		long enc = nMotorEncoder[motorC];
		wait1Msec(200);
		if(enc-nMotorEncoder[motorC] < 10)
			break;
	}

	bFloatDuringInactiveMotorPWM = true;
	motor[motorC] = 0;
	wait1Msec(500);
	bFloatDuringInactiveMotorPWM = false;
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
			move_enc_xy(buf[1]|buf[2]<<8, buf[3]|buf[4]<<8, run_opt.speed);
#ifdef MOVE_DELAY
			wait1Msec(MOVE_DELAY);
#endif
			return 5;
		case 'm':
			move_delta_xy(buf[1]|buf[2]<<8, buf[3]|buf[4]<<8, run_opt.speed);
#ifdef MOVE_DELAY
			wait1Msec(MOVE_DELAY);
#endif
			return 5;
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

void xy_to_uv(int *u, int *v, int x, int y){
	x += X_OFFSET;
	y += Y_OFFSET;

	*u = (int) (KU * (U0 + (360.0 * x / (2.0 * PI * R)) ));

	float f = y*y + L1*L1 - L2*L2;

#ifdef DEBUG_UV
	writeDebugStream("uv: %f = %d^2 + %d^2 - %d^2\n", f, y, L1, L2);
#endif

	f /= 2.0 * L1 * y;

	if(abs(f) > 0.99){
#ifdef DEBUG_UV
		writeDebugStream("uv: got cos out of bounds (%f), truncating\n", f);
#endif
		f = 0.99 * sgn(f);
	}

#ifdef DEBUG_UV
	writeDebugStream("uv: acos(%f) = %f\n", f, acos(f));
#endif

	*v = (int) (KV * (V0 - radiansToDegrees(acos(f))));
}

void uv_to_xy(int *x, int *y, int u, int v){
	u = u/KU - U0;
	v = V0 - v/KV;

	*x = (int) (2 * PI * R * u / 360.0) - X_OFFSET;

	float cos_v = cos(degreesToRadians(v));
	*y = (int) (L1*cos_v + sqrt(L2*L2 - (L1*L1)*(1 - cos_v*cos_v)) - Y_OFFSET);
}

void move_enc_xy(long x, long y, float speed){
	int u, v;
	xy_to_uv(&u, &v, x, y);
	writeDebugStream("xy(%d, %d) => uv(%d, %d)\n", x, y, u, v);
	move_enc_uv(u, v, speed);
}

void move_enc_uv(long b, long c, float speed){
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

		pu = mu * min(pu, USPEED_MAX);
		pv = mv * min(pv, VSPEED_MAX);

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
	move_enc_uv(nMotorEncoder[motorB] + db, nMotorEncoder[motorC] + dc, speed);
}

void move_delta_xy(long dx, long dy, float speed){
	int x, y;
	uv_to_xy(&x, &y, nMotorEncoder[motorB], nMotorEncoder[motorC]);
	move_enc_xy(x+dx, y+dy, speed);
}

void move_z(int up){
	if(run_state.pen_up == up)
		return;

	run_state.pen_up = up;

	// Motor encoders don't work here so this is timed
	/*
	motor[motorA] = up ? 50 : -50;
	wait1Msec(500);
	motor[motorA] = 0;
	*/
	// NVM it doesn't work anyway
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

void test_circle(int r, int N){
	for(int i=0;i<N;i++){
		int x = (r+5) + (int) (r*cos(2*PI*i/N));
		int y = (r+5) + (int) (r*sin(2*PI*i/N));
		move_enc_xy(x, y, run_opt.speed);
		wait1Msec(500);
	}
}
