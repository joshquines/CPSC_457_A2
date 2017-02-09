/******************************************************************************
    Copyright © 2012-2015 Martin Karsten

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/
#include "syscalls.h"
#include "pthread.h"

#include <unistd.h>
#include <stdio.h>

int y = 83;

static pthread_mutex_t iolock;

static void* foobar1(void*) {
  for (int j = 0; j < 500000; j++) asm("" ::: "memory");
  return &y;
}

static void* foobar2(void*) {
  pthread_mutex_lock(&iolock);
  printf("hokey\n");
  pthread_mutex_unlock(&iolock);
  for (int i = 0; i < 20; i++) {
    pthread_mutex_lock(&iolock);
    printf("pokey %d\n", i);
    pthread_mutex_unlock(&iolock);
    for (int j = 0; j < 500000; j++) asm("" ::: "memory");
  }
  pthread_mutex_lock(&iolock);
  printf("dokey\n");
  int* x = new int;              // no separate memory lock for now...
  pthread_mutex_unlock(&iolock);
  *x = 84;
  return x;
}

static void* foobar3(void*) {
  for (;;) asm("" ::: "memory");
  return nullptr;
}

extern int signum;

int main() {
//  *(int*)0xdeadbeef = 0; // test page fault handling
//  return 10 / 0; // test exception handling
  pthread_mutex_init(&iolock, nullptr);
  pthread_mutex_lock(&iolock);
  pthread_mutex_unlock(&iolock);
  pthread_t t1;
  pthread_create(&t1, nullptr, foobar1, nullptr);
  void* result;
  pthread_join(t1, &result);
  pthread_mutex_lock(&iolock);
  printf("foobar1: %i\n", *(int*)result);
  pthread_mutex_unlock(&iolock);
  pthread_t t2, t3;
  pthread_create(&t3, nullptr, foobar3, nullptr);
  pthread_create(&t2, nullptr, foobar2, nullptr);
  pthread_mutex_lock(&iolock);
  printf("foobar3 created\n");
  pthread_mutex_unlock(&iolock);
  for (int i = SyscallNum::max; i <= 100; i++) syscallStub(i);
  for (int i = 0; i < 10; i++) {
    pthread_mutex_lock(&iolock);
    printf("working %d\n", i);
    pthread_mutex_unlock(&iolock);
    for (int j = 0; j < 100000; j++) asm("" ::: "memory");
  }
  pthread_join(t2, &result);
  pthread_mutex_destroy(&iolock);         // no conflicts with t3
  printf("foobar2: %i\n", *(int*)result);
  printf("signal received: 0x%x\n", signum);
  delete (int*)result;
  printf("goodbye\n");
  return 0;
};
