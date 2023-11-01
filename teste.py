class Ue4Briedge:
    def __init__(self, host : str) -> None:
        """Starts communication's Engine.

        Args:
            host (str, optional): 
                -For set ip address from host, use ipconfig result's on host
                -For set ip address from docker network, use a ip from ping result's between containers on host
                -For set ip address from WSL, os.environ['WSL_HOST_IP'] on host.
        """        
        self.__client = MultirotorClient(host)
        self.__client.confirmConnection()
        rospy.logwarn(f"\nConnection: {self.__client.ping()}")
        
    @property
    def client(self):
        return self.__client
                
    def restart(self) -> None:
        """
        Reset the ue4 client conection.
        """        
        self.__client.reset()

        