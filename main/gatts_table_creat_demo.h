/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

 
    HRS_IDX_NB,
};

/*add a new service*/
enum
{
    IDX_SVC2,
    IDX_CHAR_A2,
    IDX_CHAR_VAL_A2,
    IDX_CHAR_CFG_A2,

    IDX_CHAR_B2,
    IDX_CHAR_VAL_B2,

    HRS_IDX_NB2,
};
