

def bytes_2_multicast_postfix(data: bytes,
                              size: int = 14) -> str:
    """
    converts an array of bytes to a string suitable as multicast group postfix
    """
    return data[-size:].hex(sep=':', bytes_per_sep=2)
