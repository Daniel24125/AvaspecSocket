
class SpectrometerException(Exception): 
    
    def __init__(self, error_name, error_msg, error_code):
        self.error_name = error_name
        self.error_msg = error_msg
        self.error_code = error_code
        super().__init__(self.error_name, self.error_msg, self.error_code)

class UnauthorizedConnection(Exception): 
    pass