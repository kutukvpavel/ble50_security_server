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

    ATTR_IDX_MEASURE_DECL,
    ATTR_IDX_MEASURE_VAL,
    ATTR_IDX_MEASURE_CCC,
    ATTR_IDX_MEASURE_DESC,
    ATTR_IDX_MEASURE_PRES,

    HRS_IDX_NB,
};
