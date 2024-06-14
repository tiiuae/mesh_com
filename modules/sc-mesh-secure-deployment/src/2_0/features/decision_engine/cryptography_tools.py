import cryptography.exceptions
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives.serialization import load_pem_private_key
from cryptography.hazmat.backends import default_backend
from cryptography import x509

# ************************************************************ Note ********************************************************************
# The code currently loads private key from a file
# HSM integration is on the way, and once done, the sign_message function should be updated to access the private key securely
# **************************************************************************************************************************************

def sign_message(message, priv_key_location, logger):
     """
    Sign a message using an ECDSA private key.

    This function takes a message and the location of an EC private key file,
    then signs the message using ECDSA with SHA256 hashing. It logs relevant information
    and errors using the provided logger.

    Parameters:
    message (bytes): The message to be signed. Must be a byte string.
    priv_key_location (str): The file path to the EC private key in PEM format.
    logger (logging.Logger): A logger instance for logging information and errors.

    Returns:
    bytes: The digital signature of the message, or None if an error occurs.
    """
     try:
        # Load EC private key from file
        with open(priv_key_location, "rb") as key_file:
            private_key = load_pem_private_key(key_file.read(), password=None, backend=default_backend())

        # Sign the message using ECDSA
        signature = private_key.sign(
            message,
            ec.ECDSA(hashes.SHA256())
        )
        return signature
     except FileNotFoundError:
         logger.info(f"Error: The file {priv_key_location} was not found.")
         return None
     except Exception as e:
         logger.info(f"An unexpected error occurred while signing the message: {e}")
         return None

def verify_signature(signature, message, pub_key_location, logger):
    """
   Verify a signature using an ECDSA public key from a certificate.

   This function takes a digital signature, a message, and the location of an EC public key
   certificate file, then verifies the signature using ECDSA with SHA256 hashing. It logs relevant
   information and errors using the provided logger.

   Parameters:
   signature (bytes): The digital signature to be verified.
   message (bytes): The original message that was signed. Must be a byte string.
   pub_key_location (str): The file path to the EC public key certificate in PEM format.
   logger (logging.Logger): A logger instance for logging information and errors.

   Returns:
   bool: True if the signature is valid, False otherwise.
    """
    # Load the public key from the EC certificate
    with open(pub_key_location, 'rb') as cert_file:
        cert = x509.load_pem_x509_certificate(cert_file.read(), backend=default_backend())
        public_key = cert.public_key()

    # Verify the signature using ECDSA
    try:
        public_key.verify(
            signature,
            message,
            ec.ECDSA(hashes.SHA256())
        )
        logger.info("Signature is valid.")
        return True
    except cryptography.exceptions.InvalidSignature:
        logger.info("Signature is invalid")
        return False
    except Exception as e:
        logger.info(f"An unexpected error occurred while verifying the signature: {e}")
        return False

def generate_signed_message(message, signature):
    """
    Generates a combined message consisting of the signature length, signature, and original message.

    This function takes a message and its digital signature, then combines them into a single byte string.
    The format of the combined message is a 2-byte signature length, followed by the signature, and then the original message.

    Parameters:
    message (bytes): The original message to be combined. Must be a byte string.
    signature (bytes): The digital signature of the message.

    Returns:
    bytes: The combined message in the format of [signature_length][signature][message].
    """
    signature_length = len(signature)
    return signature_length.to_bytes(2, 'big') + signature + message
def extract_message_sign_from_signed_message(signed_message):
    """
    Extracts the signature and the original message from a combined signed message.

    This function takes a combined signed message, extracts the length of the signature,
    then separates and returns the signature and the original message.

    Parameters:
    signed_message (bytes): The combined signed message in the format of [signature_length][signature][message].

    Returns:
    tuple: A tuple containing the signature and the original message (signature, message).
    """
    signature_length = int.from_bytes(signed_message[:2], 'big')
    signature = signed_message[2:2 + signature_length]
    message = signed_message[2 + signature_length:]
    return signature, message