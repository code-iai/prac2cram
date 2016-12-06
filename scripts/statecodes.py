
SC_BOOTING = 0
SC_IDLE = 1
SC_BUSY = 2
SC_ERROR = 3
SC_EXIT = 4
SC_UNKNOWN = 5

RC_ALLOK = 0
RC_NOTREADY = 1
RC_ROSSRVFAIL = 2

def stateName(state):
    if (SC_BOOTING == state):
        return "BOOTING"
    elif (SC_IDLE == state):
        return "IDLE"
    elif (SC_BUSY == state):
        return "BUSY"
    elif (SC_ERROR == state):
        return "ERROR"
    elif (SC_EXIT == state):
        return "EXIT"
    else:
        return "UNKNOWN"

def retcodeName(retcode):
    if (RC_ALLOK == retcode):
        return "ALLOK"
    elif (RC_NOTREADY == retcode):
        return "NOTREADY"
    elif (RC_ROSSRVFAIL == retcode):
        return "ROSSRVFAIL"
    else:
        return "UNKNOWN"

