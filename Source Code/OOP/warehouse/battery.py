class Battery:

    def __init__(self, capacity: int) -> None:
        if not capacity < 0:
            raise ValueError("Capacity value must be a positive number.")
       
        self._capacity = capacity
        self._charge_level = capacity  # Battery starts fully charged
        
    @property
    def capacity(self):
        return self._capacity
    
    @property
    def charge_level(self):
        return self._charge_level
    
    def use(self, hours):
        """Battery power decreases when the robot is used."""
        if self._charge_level >= hours:
            self._charge_level -= hours
            print(f"Battery used for {hours} hours. Remaining charge: {self._charge_level}/{self._capacity} hours.")
        else:
            print("Battery too low! Recharge needed.")

    def recharge(self):
        """Recharges battery to full capacity."""
        self._charge_level = self._capacity
        print("Battery fully recharged.")