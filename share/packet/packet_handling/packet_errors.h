/*******************************************************************************
 *
 * FILENAME: packet_errors.h
 *
 * PROJECT: RampBotHost
 *
 * ORIGINAL AUTHOR: Lyle Cheatham
 *
 * DATE: 10/6/18
 *
 * COPYRIGHT THE ORIGINAL AUTHORS, ALL RIGHTS RESERVED UNLESS OTHERWISE NOTED
 *
 *******************************************************************************/

#pragma once

enum packet_error {
    p_err_none = 0,
    p_err_undefined = 1,
    p_err_undersize,
    p_err_oversize,
    p_err_bad_crc,
    p_err_no_catch_character,

    // Leave this at end
    number_p_err
};
