
__all__ = ['LABELS_MAP', 'POSE_STATES', 'OCCLUSION_STATES', 'TRUNCATION_STATES']

LABELS_MAP = {
    'Car': 1,
    'Van': 2,
    'Truck': 3,
    'Pedestrian': 4,
    'Person (sitting)': 5,
    'Cyclist': 6,
    'Tram': 7,
    'Misc': 8,
}

POSE_STATES = {
    0: "UNSET"   ,
    1: "INTERP"  ,
    2: "LABELED" ,
}

OCCLUSION_STATES = {
    -1: "OCCLUSION_UNSET" ,
    0 : "VISIBLE"         ,
    1 : "PARTLY"          ,
    2 : "FULLY"           ,
}

TRUNCATION_STATES = {
    -1: "TRUNCATION_UNSET" ,
    0 : "IN_IMAGE"         ,
    1 : "TRUNCATED"        ,
    2 : "OUT_IMAGE"        ,
    99: "BEHIND_IMAGE"     ,
}
