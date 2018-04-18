#ifndef __MSG_RET_GEN__
#define __MSG_RET_GEN__
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

void forwardRet(uint8_t a){
    uint8_t forwardRetMsg[1+sizeof(uint8_t)]={7};
    uint8_t p=1;
    uint8_t t_a=a;
    memcpy(&(forwardRetMsg[p]), &t_a, sizeof(uint8_t));
    p+=sizeof(uint8_t);
    sendPoUART(CHRPC, p, forwardRetMsg);
}

void backwardRet(uint8_t a){
    uint8_t backwardRetMsg[1+sizeof(uint8_t)]={8};
    uint8_t p=1;
    uint8_t t_a=a;
    memcpy(&(backwardRetMsg[p]), &t_a, sizeof(uint8_t));
    p+=sizeof(uint8_t);
    sendPoUART(CHRPC, p, backwardRetMsg);
}

void leftRet(uint8_t a){
    uint8_t leftRetMsg[1+sizeof(uint8_t)]={9};
    uint8_t p=1;
    uint8_t t_a=a;
    memcpy(&(leftRetMsg[p]), &t_a, sizeof(uint8_t));
    p+=sizeof(uint8_t);
    sendPoUART(CHRPC, p, leftRetMsg);
}

void rightRet(uint8_t a){
    uint8_t rightRetMsg[1+sizeof(uint8_t)]={10};
    uint8_t p=1;
    uint8_t t_a=a;
    memcpy(&(rightRetMsg[p]), &t_a, sizeof(uint8_t));
    p+=sizeof(uint8_t);
    sendPoUART(CHRPC, p, rightRetMsg);
}

void checkProgressRet(uint16_t a){
    uint8_t checkProgressRetMsg[1+sizeof(uint16_t)]={11};
    uint8_t p=1;
    uint16_t t_a=MHTONS(a);
    memcpy(&(checkProgressRetMsg[p]), &t_a, sizeof(uint16_t));
    p+=sizeof(uint16_t);
    sendPoUART(CHRPC, p, checkProgressRetMsg);
}

void stopRet(uint8_t a){
    uint8_t stopRetMsg[1+sizeof(uint8_t)]={12};
    uint8_t p=1;
    uint8_t t_a=a;
    memcpy(&(stopRetMsg[p]), &t_a, sizeof(uint8_t));
    p+=sizeof(uint8_t);
    sendPoUART(CHRPC, p, stopRetMsg);
}

#endif //__MSG_RET_GEN__
