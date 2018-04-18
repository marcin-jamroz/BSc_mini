#ifndef __MSG_INTERPRETER__
#define __MSG_INTERPRETER__
/*
   This file is automatically generated. Do not modify it directly,
   instead modifiy the 'wezel_2.xml' file and re-run RPCCodeGen tool again!

   This file is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program. If not, see http://www.gnu.org/licenses.

   The methodology, constrcutions and some use cases of this tool and
   NanoRPC protocol was presented in: A.Pruszkowski, "Maszynowa
   generacja oprogramowania dla dwumikrokontrolerowych wezlow Internetu
   Rzeczy", in procidings PRZEGLAD TELEKOMUNIKACYJNY, 2015/8-9,
   ISSN 1230-3496, e-ISSN 2449-7487, Pages: 861-868,
   DOI:10.15199/59.2015.8-9.26

   Remarks: The memory replication mechanism was addad in July 2016.
   This mechanism utilise the proxy protocol used in the SmartCampus
   experiment. The experiment was desribed in: T.Tajmajer at al,
   "Node/Proxy portability: Designing for the two lives of your next
   WSAN middleware", Publicated in: Journal of Systems and Software,
   Volume 117, July 2016, Pages 366-383, Elsevier,
   https://doi.org/10.1016/j.jss.2016.03.035

   The below part of this file was created for
   user: mjamroz at 2017-12-03 16:34:21 by RPCCodeGen.
   The RPCCodeGen tool was made by Aleksander Pruszkowski (apruszko@o2.pl)

   Developing history:
   2016.07.21 - Improved memory replication mechanism (the memory contents
                migration is possible in the two directions).
   2016.04.29 - A few minor improvements (major cleaning of the python code).
   2016.07.01 - Added memory replication mechanism (the memory contenst
                migration is realised from master part to slave part only).
   2015.09.20 - A few minor improvements (preprocessing source code).
   2015.04.08 - First working version, support only simple types of
                arguments provided to services during invocation.
*/

#define CHRPC        0x41

#ifndef MEM_REP_CHUNK
#define MEM_REP_CHUNK        15
#endif
#ifndef MAX_MEM_REP_BUFOR
#define MAX_MEM_REP_BUFOR		128
#endif

#define CONST_FORWARD            0x01
#define CONST_BACKWARD            0x02
#define CONST_LEFT            0x03
#define CONST_RIGHT            0x04
#define CONST_CHECKPROGRESS            0x05
#define CONST_STOP            0x06
#define MEM_REPLICATION_DATA_PUT_ACK        0xfa
#define MEM_REPLICATION_DATA_PUT        0xfb
#define MEM_REPLICATION_DATA_REQ        0xfe
#define MEM_REPLICATION_DATA_RES        0xff

//Prototypes list of returning function
void forwardSrv(uint16_t a1, uint8_t a2);
void backwardSrv(uint16_t a1, uint8_t a2);
void leftSrv(uint16_t a1, uint8_t a2);
void rightSrv(uint16_t a1, uint8_t a2);
void checkProgressSrv(void);
void stopSrv(void);
//------------------
//void forwardRet(uint8_t a);
//void backwardRet(uint8_t a);
//void leftRet(uint8_t a);
//void rightRet(uint8_t a);
//void checkProgressRet(uint16_t a);
//void stopRet(uint8_t a);
//end of list of prototypes of returning function

#define PACK_STRUCT	__attribute__((packed))
struct _nanoRPC_state {
  union _services_args {
    struct _forwardSrv {
      uint16_t a1;
      uint8_t a2;
    } PACK_STRUCT forwardSrv;
    struct _backwardSrv {
      uint16_t a1;
      uint8_t a2;
    } PACK_STRUCT backwardSrv;
    struct _leftSrv {
      uint16_t a1;
      uint8_t a2;
    } PACK_STRUCT leftSrv;
    struct _rightSrv {
      uint16_t a1;
      uint8_t a2;
    } PACK_STRUCT rightSrv;
    struct _checkProgressSrv {
    } PACK_STRUCT checkProgressSrv;
    struct _stopSrv {
    } PACK_STRUCT stopSrv;
  } services_args;
  int nanorpc_service_code;
  uint8_t mem_rep_buffer[MAX_MEM_REP_BUFOR];
  uint32_t mem_rep_slave_buffer_ptr;
  uint32_t mem_rep_size;
  uint32_t master_addr;
} nanoRPC_state;

void finlise_srv(void) {
  switch (nanoRPC_state.nanorpc_service_code) {
    case CONST_FORWARD: {
        forwardSrv(nanoRPC_state.services_args.forwardSrv.a1, nanoRPC_state.services_args.forwardSrv.a2);
        break;
      }
    case CONST_BACKWARD: {
        backwardSrv(nanoRPC_state.services_args.backwardSrv.a1, nanoRPC_state.services_args.backwardSrv.a2);
        break;
      }
    case CONST_LEFT: {
        leftSrv(nanoRPC_state.services_args.leftSrv.a1, nanoRPC_state.services_args.leftSrv.a2);
        break;
      }
    case CONST_RIGHT: {
        rightSrv(nanoRPC_state.services_args.rightSrv.a1, nanoRPC_state.services_args.rightSrv.a2);
        break;
      }
    case CONST_CHECKPROGRESS: {
        checkProgressSrv();
        break;
      }
    case CONST_STOP: {
        stopSrv();
        break;
      }
  }
}

void mem_replication_data_put(uint32_t master_addr, uint8_t size, uint8_t *d) {
  uint8_t mem_replicationMsg[1 + sizeof(uint32_t) + sizeof(uint8_t) + MEM_REP_CHUNK] = {MEM_REPLICATION_DATA_PUT};
  uint8_t p = 1;
  uint32_t t_a1 = MHTONL(nanoRPC_state.master_addr + nanoRPC_state.mem_rep_slave_buffer_ptr);
  uint8_t t_a2 = size;
  memcpy(&(mem_replicationMsg[p]), &t_a1, sizeof(uint32_t));
  p += sizeof(uint32_t);
  memcpy(&(mem_replicationMsg[p]), &t_a2, sizeof(uint8_t));
  p += sizeof(uint8_t);
  memcpy(&(mem_replicationMsg[p]), d, size);
  p += size;
  sendPoUART(CHRPC, p, mem_replicationMsg);
}
void mem_replication_data_put_ack(uint8_t t_a1) {
  nanoRPC_state.mem_rep_slave_buffer_ptr += t_a1;
  if (nanoRPC_state.mem_rep_size > nanoRPC_state.mem_rep_slave_buffer_ptr) { //nastepny fragment
    if (nanoRPC_state.mem_rep_size - nanoRPC_state.mem_rep_slave_buffer_ptr < MEM_REP_CHUNK)
      mem_replication_data_put(nanoRPC_state.master_addr + nanoRPC_state.mem_rep_slave_buffer_ptr, nanoRPC_state.mem_rep_size - nanoRPC_state.mem_rep_slave_buffer_ptr, &(nanoRPC_state.mem_rep_buffer[nanoRPC_state.mem_rep_slave_buffer_ptr]));
    else
      mem_replication_data_put(nanoRPC_state.master_addr + nanoRPC_state.mem_rep_slave_buffer_ptr, MEM_REP_CHUNK, &(nanoRPC_state.mem_rep_buffer[nanoRPC_state.mem_rep_slave_buffer_ptr]));
  } else {
    switch (nanoRPC_state.nanorpc_service_code) { //finalise service
        break;
    }
  }
}
void mem_replication_data_req(uint32_t size) {
  uint8_t mem_replicationMsg[1 + sizeof(uint32_t) + sizeof(uint8_t)] = {MEM_REPLICATION_DATA_REQ};
  uint8_t p = 1;
  uint32_t t_a1 = MHTONL(nanoRPC_state.master_addr + nanoRPC_state.mem_rep_slave_buffer_ptr);
  uint8_t t_a2 = size;
  memcpy(&(mem_replicationMsg[p]), &t_a1, sizeof(uint32_t));
  p += sizeof(uint32_t);
  memcpy(&(mem_replicationMsg[p]), &t_a2, sizeof(uint8_t));
  p += sizeof(uint8_t);
  sendPoUART(CHRPC, p, mem_replicationMsg);
}
void mem_replication_data_res(uint8_t t_a1, uint8_t *t_a2) {
  //FIXME!!! Check if new write is not outside the buffer!
  if (nanoRPC_state.mem_rep_slave_buffer_ptr + t_a1 > MAX_MEM_REP_BUFOR) {
    memcpy(&(nanoRPC_state.mem_rep_buffer[nanoRPC_state.mem_rep_slave_buffer_ptr]), t_a2, MAX_MEM_REP_BUFOR - nanoRPC_state.mem_rep_slave_buffer_ptr);
    nanoRPC_state.mem_rep_slave_buffer_ptr += MAX_MEM_REP_BUFOR - nanoRPC_state.mem_rep_slave_buffer_ptr;
    finlise_srv();
  } else {
    memcpy(&(nanoRPC_state.mem_rep_buffer[nanoRPC_state.mem_rep_slave_buffer_ptr]), t_a2, t_a1);
    nanoRPC_state.mem_rep_slave_buffer_ptr += t_a1;
    if (nanoRPC_state.mem_rep_slave_buffer_ptr < nanoRPC_state.mem_rep_size) {
      if (nanoRPC_state.mem_rep_size - nanoRPC_state.mem_rep_slave_buffer_ptr > MEM_REP_CHUNK) {
        mem_replication_data_req(MEM_REP_CHUNK);
      } else {
        mem_replication_data_req(nanoRPC_state.mem_rep_size - nanoRPC_state.mem_rep_slave_buffer_ptr);
      }
    } else {
      finlise_srv();
    }
  }
}
//Services of slave part:
void forwardSrv_memory_rep_s2m(uint16_t a1, uint8_t a2) {
  nanoRPC_state.services_args.forwardSrv.a1 = a1;
  nanoRPC_state.services_args.forwardSrv.a2 = a2;
  nanoRPC_state.nanorpc_service_code = CONST_FORWARD;
  finlise_srv();
}
void backwardSrv_memory_rep_s2m(uint16_t a1, uint8_t a2) {
  nanoRPC_state.services_args.backwardSrv.a1 = a1;
  nanoRPC_state.services_args.backwardSrv.a2 = a2;
  nanoRPC_state.nanorpc_service_code = CONST_BACKWARD;
  finlise_srv();
}
void leftSrv_memory_rep_s2m(uint16_t a1, uint8_t a2) {
  nanoRPC_state.services_args.leftSrv.a1 = a1;
  nanoRPC_state.services_args.leftSrv.a2 = a2;
  nanoRPC_state.nanorpc_service_code = CONST_LEFT;
  finlise_srv();
}
void rightSrv_memory_rep_s2m(uint16_t a1, uint8_t a2) {
  nanoRPC_state.services_args.rightSrv.a1 = a1;
  nanoRPC_state.services_args.rightSrv.a2 = a2;
  nanoRPC_state.nanorpc_service_code = CONST_RIGHT;
  finlise_srv();
}
void checkProgressSrv_memory_rep_s2m() {
  nanoRPC_state.nanorpc_service_code = CONST_CHECKPROGRESS;
  finlise_srv();
}
void stopSrv_memory_rep_s2m() {
  nanoRPC_state.nanorpc_service_code = CONST_STOP;
  finlise_srv();
}
//End of services of slave part

void recvPoUART(uint8_t c, uint8_t l, uint8_t *r) {
  if (c == CHRPC) {
    switch (r[0]) {
      case CONST_FORWARD: {	 //forward()
          uint8_t p = 1;
          uint16_t t_a1;
          uint8_t t_a2;
          if (l != 4)
            NANORPC_FRAME_ERROR(CONST_FORWARD);
          memcpy(&t_a1, &(r[p]), sizeof(uint16_t));
          p += sizeof(uint16_t);
          memcpy(&t_a2, &(r[p]), sizeof(uint8_t));
          p += sizeof(uint8_t);
          forwardSrv_memory_rep_s2m(MNTOHS(t_a1), t_a2);
          break;
        }
      case CONST_BACKWARD: {	 //backward()
          uint8_t p = 1;
          uint16_t t_a1;
          uint8_t t_a2;
          if (l != 4)
            NANORPC_FRAME_ERROR(CONST_BACKWARD);
          memcpy(&t_a1, &(r[p]), sizeof(uint16_t));
          p += sizeof(uint16_t);
          memcpy(&t_a2, &(r[p]), sizeof(uint8_t));
          p += sizeof(uint8_t);
          backwardSrv_memory_rep_s2m(MNTOHS(t_a1), t_a2);
          break;
        }
      case CONST_LEFT: {	 //left()
          uint8_t p = 1;
          uint16_t t_a1;
          uint8_t t_a2;
          if (l != 4)
            NANORPC_FRAME_ERROR(CONST_LEFT);
          memcpy(&t_a1, &(r[p]), sizeof(uint16_t));
          p += sizeof(uint16_t);
          memcpy(&t_a2, &(r[p]), sizeof(uint8_t));
          p += sizeof(uint8_t);
          leftSrv_memory_rep_s2m(MNTOHS(t_a1), t_a2);
          break;
        }
      case CONST_RIGHT: {	 //right()
          uint8_t p = 1;
          uint16_t t_a1;
          uint8_t t_a2;
          if (l != 4)
            NANORPC_FRAME_ERROR(CONST_RIGHT);
          memcpy(&t_a1, &(r[p]), sizeof(uint16_t));
          p += sizeof(uint16_t);
          memcpy(&t_a2, &(r[p]), sizeof(uint8_t));
          p += sizeof(uint8_t);
          rightSrv_memory_rep_s2m(MNTOHS(t_a1), t_a2);
          break;
        }
      case CONST_CHECKPROGRESS: {	 //checkProgress()
          uint8_t p = 1;
          if (l != 1)
            NANORPC_FRAME_ERROR(CONST_CHECKPROGRESS);
          checkProgressSrv_memory_rep_s2m();
          break;
        }
      case CONST_STOP: {	 //stop()
          uint8_t p = 1;
          if (l != 1)
            NANORPC_FRAME_ERROR(CONST_STOP);
          stopSrv_memory_rep_s2m();
          break;
        }
      case MEM_REPLICATION_DATA_PUT_ACK: {
          uint8_t p = 1;
          uint8_t t_a1;
          if (l != 2)
            NANORPC_FRAME_ERROR(MEM_REPLICATION_DATA_PUT_ACK);
          memcpy(&t_a1, &(r[p]), sizeof(uint8_t));
          mem_replication_data_put_ack(t_a1);
          break;
        }
      case MEM_REPLICATION_DATA_RES: {
          uint8_t p = 1;
          uint8_t t_a1;	//len of mem rep chunk
          uint8_t *t_a2;  //ptr to mem rep chunk contents
          if (l <= 2)
            NANORPC_FRAME_ERROR(MEM_REPLICATION_DATA_RES);
          memcpy(&t_a1, &(r[p]), sizeof(uint8_t));
          if (t_a1 > MEM_REP_CHUNK)
            NANORPC_FRAME_ERROR(MEM_REPLICATION_DATA_RES);
          p += sizeof(uint8_t);
          t_a2 = &(r[p]);
          mem_replication_data_res(t_a1, t_a2);
          break;
        }
    }
  }
}
#endif //__MSG_INTERPRETER__
