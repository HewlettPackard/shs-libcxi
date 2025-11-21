#!/bin/bash
# Copyright 2025 Hewlett Packard Enterprise Development LP

# Headshell Device Helper script core functions on Cassini
#
# To simplify the commandline debugging of reading/writing to Headshell transceivers
# these helper scripts were created in order to automate the hardware reference
# lookups and prevent i2c collisions (with flock for exclusive access)
#
# The user must include the headshell jack number on the command line so the
# scripts know which cassini to access.
#

# Determine command name / script name
CMD_NAME=$(basename $0)

# All our Headshell devices have the same I2C slave address
DEVADDR=0x50

I2CBUS=""

# Get I2CBUS number from Headshell value
function get_i2cbus_from_dev {
    local hs=$1
    local regex_str="i2c-([0-9]+).*cxi${hs} headshell"

    if ! command -v i2cdetect >/dev/null 2>&1; then
        echo Error: 'i2cdetect' not found. Please install i2c-tools
        help_exit
    fi

    while IFS= read -r line; do
        if [[ "$line" =~ $regex_str ]]; then
            I2CBUS="${BASH_REMATCH[1]}"
        fi
    done <<< "$(i2cdetect -l)"

    if [[ $I2CBUS == "" ]]; then
        echo Error: Unable to find I2C bus for Headshell cxi${hs}
        help_exit
    fi
}

# Validate global $PAGE parameter input
function validate_page_parm {
    grep -q -e '^0[xX][[:xdigit:]]\+$' -e '^[[:digit:]]\+$' <<< $PAGE || PAGE=-1
    # Attempt conversion (might be in hex)
    PAGE=$(( PAGE ))
    if [ "$PAGE" -lt 0 -o "$PAGE" -gt 255 ] ; then
        echo "-p option requires a page number (0-255)"
        help_exit
    fi
}

# Validate global $BANK parameter input
function validate_bank_parm {
    grep -q -e '^0[xX][[:xdigit:]]\+$' -e '^[[:digit:]]\+$' <<< $BANK || BANK=-1
    # Attempt conversion (might be in hex)
    BANK=$(( BANK ))
    if [ "$BANK" -lt 0 -o "$BANK" -gt 255 ] ; then
        echo "-b option requires a numeric bank number (0-255)"
        help_exit
    fi
}

# Test for valid DEV number
function dev_index_valid {
    if [[ "$1" =~ cxi([0-9]+) ]]; then
        DEV="${BASH_REMATCH[1]}"
    else
        DEV="-1"
    fi
    [ "$DEV" -ge 0 -a "$DEV" -le 8 ]
}

# Carry out the hs i2c transaction by acquiring the flock on 
# file descriptor 200 and selecting the necessary i2c bus before
# executing the command. The flock is used to prevent simultaneous
# access to multiple modules. It is an advisory lock.
#
# Typical invocation (with file descriptor 200):
#   hs_i2c_transact [hs] [i2cbus] [page] [bank] [i2ccommand] {parameters...} 200>/dev/i2c-${I2CBUS}
#
#   hs         = the hs reference
#   i2cbus       = the i2cbus number for the hs module
#   page         = page select index (ignored if less than 0)
#   bank         = bank select index (ignored if less than 0)
#   i2ccommand   = the i2c command (i2cget, i2cset, i2cdump, etc)
#   {parameters} = optional parameters to include with the i2c command
#
# Return value is
#   99 if the lock was not acquired
#    1 if the MODSEL GPIO could not be asserted
#          otherwise it is the return value from the i2c command
#
function hs_i2c_transact {
    local hs=$1
    local i2cbus=$2
    local page=$3
    local bank=$4
    local rc

    # Shift 4 so '$@' will be the i2c command and trailing parameters
    shift 4

    # Try to get the lock based on the file associated with descriptor 200
    # or else immediately return 99 to indicate the lock was not acquired
    flock -n 200 || return 99

    #echo hs=$hs bus=$i2cbus cmd=$@

    # Possibly write the bank & page select bytes first
    if [ "$bank" -ge 0 ] ; then
        if [ "$page" -ge 0 ] ; then
            # Write a word to set both in one transaction
            if ! i2cset -y -f $i2cbus $DEVADDR 126 $(( page * 256 + bank )) w ; then
                echo "Error: Failed to set bank/page select bytes"
            fi
        else
            # Write just the bank select byte
            if ! i2cset -y -f $i2cbus $DEVADDR 126 $bank ; then
                echo "Error: Failed to set bank select byte"
            fi
        fi
        sleep 0.001
    # Possibly write the page select byte first
    elif [ "$page" -ge 0 ] ; then
        if ! i2cset -y -f $i2cbus $DEVADDR 127 $page ; then
            echo "Error: Failed to set page select byte"
        fi
        sleep 0.001
    fi

    # Do the requested transaction
    $@
    rc=$?

    return $rc
}

