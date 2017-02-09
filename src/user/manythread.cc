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

#include <iostream>

static pthread_mutex_t iolock;

static void* task(void* x) {
  int id = getcid();
  for (;;) {
    pthread_mutex_lock(&iolock);
    std::cout << *(char*)x << id << ' ';
    std::cout.flush();
    pthread_mutex_unlock(&iolock);
    int newid = id;
    int cnt = 0;
    while (newid == id && cnt < 100000) {
      asm volatile("pause");
      newid = getcid();
      cnt += 1;
    }
    id = newid;
  }
  return nullptr;
}

int main() {
  pthread_mutex_init( &iolock, nullptr );
  pthread_t t;
  pthread_create(&t, nullptr, task, (void*)"A");
  pthread_create(&t, nullptr, task, (void*)"B");
  pthread_create(&t, nullptr, task, (void*)"C");
  pthread_create(&t, nullptr, task, (void*)"D");
  pthread_create(&t, nullptr, task, (void*)"E");
  pthread_create(&t, nullptr, task, (void*)"F");
  pthread_create(&t, nullptr, task, (void*)"G");
  pthread_create(&t, nullptr, task, (void*)"H");
  pthread_create(&t, nullptr, task, (void*)"I");
  pthread_create(&t, nullptr, task, (void*)"J");
  pthread_create(&t, nullptr, task, (void*)"K");
  pthread_create(&t, nullptr, task, (void*)"L");
  pthread_create(&t, nullptr, task, (void*)"M");
  pthread_create(&t, nullptr, task, (void*)"N");
  pthread_create(&t, nullptr, task, (void*)"O");
  pthread_create(&t, nullptr, task, (void*)"P");
  pthread_create(&t, nullptr, task, (void*)"Q");
  pthread_create(&t, nullptr, task, (void*)"R");
  pthread_create(&t, nullptr, task, (void*)"S");
  pthread_create(&t, nullptr, task, (void*)"T");
  pthread_create(&t, nullptr, task, (void*)"U");
  pthread_create(&t, nullptr, task, (void*)"V");
  pthread_create(&t, nullptr, task, (void*)"W");
  pthread_create(&t, nullptr, task, (void*)"X");
  pthread_create(&t, nullptr, task, (void*)"Y");
  task( (void*)"Z");
  return 0;
};
