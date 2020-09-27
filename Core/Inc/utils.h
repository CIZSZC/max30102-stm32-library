/*
 * utils.h
 *
 *  Created on: 18 сент. 2020 г.
 *      Author: Dmitry Ses
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "string.h"
#include "stdio.h"
#include "assert.h"

/*ANSI colors*/
#define TEXT_COLOR_DEFAULT      "\033[0m"
#define TEXT_COLOR_RED          "\033[31m"
#define TEXT_COLOR_GREEN        "\033[32m"
#define TEXT_COLOR_YELLOW       "\033[33m"
#define TEXT_COLOR_BLUE         "\033[34m"
#define TEXT_COLOR_MAGENTA      "\033[35m"
#define TEXT_COLOR_CYAN         "\033[36m"
#define TEXT_COLOR_GRAY         "\033[37m"

#define CLEAR_TERMINAL "\033c"

/**
 * \brief Check for assertion
 */
#define POINTER_NULL(_ptr) _ptr==NULL ? 0 : 1

/**
 * \brief Check pointer macro
 *
 * Checks if pointer is not NULL.
 * If NULL then assert program
 *
 * \param _ptr Pointer to check
 */
#define CHECK_PTR(_ptr) do{  \
    if(_ptr == NULL){   \
        printf(TEXT_COLOR_RED "\r\n");\
        assert(POINTER_NULL(_ptr)); \
        goto exit;  \
    }   \
}while(0)

/**
 * \brief Continue on success code
 *
 * Continue execution if received HAL_OK code
 * otherwise go to label "exit"
 *
 * \param _retCode return code to check
 */
#define CHECK_SUCCESS(_retCode) do{  \
    HAL_StatusTypeDef _tempRetCode = _retCode;  \
    if(_tempRetCode != HAL_OK){ \
        retCode = _tempRetCode; \
        goto exit;  \
    }   \
}while(0)

/**
 * \brief prints HAL_Status code
 *
 * \param[_retCode]
 */
#define PRINT_RET(_retCode)do{  \
    switch(_retCode){   \
    case HAL_OK:    \
        printf("%s returned HAL_OK\r\n", __func__); \
        break;  \
    case HAL_ERROR: \
        printf("%s returned HAL_ERROR\r\n", __func__);  \
        break;  \
    case HAL_BUSY:  \
        printf("%s returned HAL_BUSY\r\n", __func__);   \
        break;  \
    case HAL_TIMEOUT:  \
        printf("%s returned HAL_TIMEOUT\r\n", __func__);    \
        break;  \
    default:    \
        printf("%s returned unexpected value\r\n", __func__);   \
        break;  \
    }   \
}while(0)

#define ERROR_RETURN_CODE 0
#define BUSY_RETURN_CODE 0
#define TIMEOUT_RETURN_CODE 0
#define UNEXPECTED_RETURN_CODE 0

#define CHECK_RET(_retCode) do{ \
    switch(_retCode){   \
    case HAL_OK:    \
        break;  \
    case HAL_ERROR: \
        printf(TEXT_COLOR_RED "Returned HAL_ERROR\r\n");  \
        assert(ERROR_RETURN_CODE);   \
        break;  \
    case HAL_BUSY:  \
        printf(TEXT_COLOR_RED "%s returned HAL_BUSY\r\n", __func__);   \
        assert(BUSY_RETURN_CODE);   \
        break;  \
    case HAL_TIMEOUT:  \
        printf(TEXT_COLOR_RED "%s returned HAL_TIMEOUT\r\n", __func__);    \
        assert(TIMEOUT_RETURN_CODE);   \
        break;  \
    default:    \
        printf(TEXT_COLOR_RED "%s returned unexpected value\r\n", __func__);   \
        assert(UNEXPECTED_RETURN_CODE);   \
        break;  \
    }   \
}while(0)

#define PRINT_ERROR_CODE(_retCode)

#endif /* INC_UTILS_H_ */
