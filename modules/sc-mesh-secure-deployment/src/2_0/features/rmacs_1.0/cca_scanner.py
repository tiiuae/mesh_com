import subprocess
import os
from typing import TextIO, Tuple

class CCAScan:
    '''
    A class representing Clear Channel Assessment(CCA) Scan,
    responsible for collecting CCA data from Radio which support CCA scan.
    
    Halow radio supports CCA scan using cli_app provided by newracom.
    cli_app is designed to support full band scan only.
    
    Methods:
    get_driver : List the driver which support CCA scan.
    initialize_scan :  Initialize the CCA scan to check driver and cli_app support.
    execute_scan : Execute CCA scan using command line application called cli_app provided by newracom.
    scan_report : Parse the CCA scan report to identify the interference in the interested or operating frequency,
                  and find the best optimal frequency/channel among the available channels. 
    file_open :  To open CCA scan report.
    file_close : To close the CCA scan report.
      
    '''
    def __init__(self):
        
        # Determine the path to the current working directory
        current_dir = os.getcwd()
        # Filename to store the cca scan report 
        filename = "cca_report.txt"
        # Create the file path 
        self.cca_report = os.path.join(current_dir, filename)
        # Ensure the directory exists
        os.makedirs(os.path.dirname(self.cca_report), exist_ok=True)
        # Interference detection
        self.detection = False
        
    
    def get_driver(self) -> str:
        # Get the driver which support CCA scan
        driver = "halow1"
        return driver
    
    def initialize_scan(self, driver: str) -> None:
        # Check the driver which supports CCA scan
        if driver == self.get_driver():
        # Define the command to check the availablity of cli_app
            command = ['cli_app','show version']
            try:
                process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                print(f"The cli_app is available. Version info:{process.stdout.decode.strip()}")
            except FileNotFoundError as e:
                raise Exception(f"Error: {e}") from e
        else:
            raise Exception(f"Invalid driver: {driver}")
        
    def execute_scan(self, driver: str, cur_freq: float) -> None:
        if driver == "halow1":
        # Define the command to collect CCA data 
            command = ['cli_app','show self_config US 1m 10']
            try:
                with open(self.cca_report, 'w') as output_file:
                    process = subprocess.Popen(command, stdout=output_file, stderr=subprocess.PIPE, text=True)
                    process.wait()
            #except subprocess.CalledProcessError as e:
                #print(f"Error: {e}")
            except FileNotFoundError as e:
                raise Exception(f"Error: {e}") from e
            
            # Call the scan report to parse the cca report 
            self.scan_report(self.cca_report, self.cur_freq)

        else:
            raise Exception(f"Invalid driver: {driver}")
        
    def scan_report(self, cca_report: str, cur_freq: float, interference_threshold: float) -> Tuple[bool,int]:
        # Read the CCA scan report
        try:
            cca_data = self.file_open(cca_report)
            data = cca_data.read()
            cca_values = data.strip().splitlines()
            parsed_data = []
            # Iterate over each line
            for value in cca_values:
                # Skip lines that do not contain data
                if value.startswith('--'):
                    parts = value.split()
                    #print("the len of parts",len(parts))
                    if len(parts) == 5:
                        # Check the cur_freq's cca value for interference detection
                        if (float(parts[1]) == cur_freq):
                            print("Calculate cca value:")
                            self.cca_value, self.unit = parts[3].split('%') 
                            if (self.cca_value > interference_threshold):
                                print(f"Interference is detected at operating frequency [{self.cur_freq}]")
                                self.detection = True
                            else:
                                self.detection = False 
                    else:
                        print("Expected cca report format is missing!")
                        
                elif value.startswith('[Optimal freq.]'):
                    parts = value.split()
                    #print("the len of parts",len(parts))
                    if len(parts) == 6:
                        # Store the optimal frequency
                        self.optimal_freq = parts[2]
            return  self.detection, self.optimal_freq
        except FileNotFoundError:
            print("File not found. Make sure the file exists.")
        except PermissionError:
            print("Permission error. Check if you have the necessary permissions to access the file.")
        except Exception as e:
            print(f"{e}")       
            
        
        
        
    @staticmethod
    def file_close(file_pointer: TextIO) -> None:
        """
        Close the CCA scan report

        param: Typed version of the return of open() in Text mode
        """
        file_pointer.close()

    @staticmethod
    def file_open(file_path: str) -> TextIO:
    
        #Open CCA scan report.
    
       if not os.path.exists(file_path):
            raise FileNotFoundError("File not found.")
       else:
            return open(file_path, 'r')
        
        

    
