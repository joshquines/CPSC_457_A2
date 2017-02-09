extern int LockTest();
extern int TcpTest();
extern int Experiments();
extern int InitProcess();

static void UserMain() {
  LockTest();
  TcpTest();
  Experiments();
  InitProcess();
}
