class DynamicMap(object):

    def __init__(self):
        self.static_map = None
        self.dynamic_map = None

        pass

    def setup(self):
        pass
    
    def update_dynamic_map(self):
        
        self.update_in_junction()
        self.update_ego_vehicle_index()
        self.update_surrounding_vehicle_in_lanes()
        self.update_distance_to_next_lane()

    def update_in_junction(self):
        pass

    def update_ego_vehicle_index(self):
        pass

    def update_surrounding_vehicle_in_lanes(self):
        pass

    def update_distance_to_next_lane(self):
        pass
