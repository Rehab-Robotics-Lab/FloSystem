"""Getting Started Example for Python 2.7+/3.3+"""
from __future__ import print_function
from contextlib import closing
from botocore.exceptions import BotoCoreError, ClientError
import os
import sys
import subprocess
from tempfile import gettempdir
from boto3 import Session

# Create a client using the credentials and region defined in the [adminuser]
# section of the AWS credentials file (~/.aws/credentials).
SESSION = Session(profile_name="flo")
POLLY = SESSION.client("polly")

try:
    # Request speech synthesis
    RESPONSE = POLLY.synthesize_speech(Text="Hello world!", OutputFormat="mp3",
                                       VoiceId="Joanna")
except (BotoCoreError, ClientError) as error:
    # The service returned an error, exit gracefully
    print(error)
    sys.exit(-1)

# Access the audio stream from the response
if "AudioStream" in RESPONSE:
    # Note: Closing the stream is important as the service throttles on the
    # number of parallel connections. Here we are using contextlib.closing to
    # ensure the close method of the stream object will be called automatically
    # at the end of the with statement's scope.
    with closing(RESPONSE["AudioStream"]) as stream:
        OUTPUT = os.path.join(gettempdir(), "speech.mp3")

        try:
            # Open a file for writing the output as a binary stream
            with open(OUTPUT, "wb") as file_handle:
                file_handle.write(stream.read())
        except IOError as error:
            # Could not write to file, exit gracefully
            print(error)
            sys.exit(-1)

else:
    # The response didn't contain audio data, exit gracefully
    print("Could not stream audio")
    sys.exit(-1)

# Play the audio using the platform's default player
if sys.platform == "win32":
    os.startfile(OUTPUT)
else:
    # the following works on Mac and Linux. (Darwin = mac, xdg-open = linux).
    OPENER = "open" if sys.platform == "darwin" else "xdg-open"
    subprocess.call([OPENER, OUTPUT])
