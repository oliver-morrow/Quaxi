from enum import Enum

NODES_DATA = [
    {
        "name": "PondsideAve.:QuackSt",
        "x": 28,
        "y": 329,
        "connections": [
            {"neighbor": "MigrationAve.:QuackSt.", "distance": 301},
            {"neighbor": "BreadcrumbAve.:WaddleWay", "distance": 404},
            {"neighbor": "PondsideAve.:WaddleWay", "distance": 223}
        ]
    },
    {
        "name": "MigrationAve.:QuackSt.",
        "x": 29,
        "y": 135,
        "connections": [
            {"neighbor": "PondsideAve.:QuackSt", "distance": 301},
            {"neighbor": "AquaticAve.:WaddleWay", "distance": 277},
            {"neighbor": "MigrationAve.:WaddleWay", "distance": 146}
        ]
    },
    {
        "name": "AquaticAve.:WaddleWay",
        "x": 129,
        "y": 29,
        "connections": [
            {"neighbor": "MigrationAve.:QuackSt.", "distance": 277},
            {"neighbor": "MigrationAve.:WaddleWay", "distance": 153},
            {"neighbor": "AquaticAve.:WaterfoulWay", "distance": 128}
        ]
    },
    {
        "name": "MigrationAve.:WaddleWay",
        "x": 129,
        "y": 135,
        "connections": [
            {"neighbor": "AquaticAve.:WaddleWay", "distance": 153},
            {"neighbor": "MigrationAve.:QuackSt.", "distance": 146},
            {"neighbor": "PondsideAve.:WaddleWay", "distance": 212},
            {"neighbor": "MigrationAve.:WaterfoulWay", "distance": 124}
        ]
    },
    {
        "name": "PondsideAve.:WaddleWay",
        "x": 157,
        "y": 266,
        "connections": [
            {"neighbor": "MigrationAve.:WaddleWay", "distance": 212},
            {"neighbor": "PondsideAve.:QuackSt", "distance": 223},
            {"neighbor": "BreadcrumbAve.:WaddleWay", "distance": 298},
            {"neighbor": "PondsideAve.:WaterfoulWay", "distance": 95}
        ]
    },
    {
        "name": "BreadcrumbAve.:WaddleWay",
        "x": 181,
        "y": 459,
        "connections": [
            {"neighbor": "PondsideAve.:WaddleWay", "distance": 298},
            {"neighbor": "PondsideAve.:QuackSt", "distance": 404},
            {"neighbor": "BreadcrumbAve.:TheCircle", "distance": 214}
        ]
    },
    {
        "name": "AquaticAve.:WaterfoulWay",
        "x": 213,
        "y": 29,
        "connections": [
            {"neighbor": "AquaticAve.:WaddleWay", "distance": 128},
            {"neighbor": "MigrationAve.:WaterfoulWay", "distance": 158},
            {"neighbor": "AquaticAve.:FeatherSt.", "distance": 133}
        ]
    },
    {
        "name": "MigrationAve.:WaterfoulWay",
        "x": 213,
        "y": 135,
        "connections": [
            {"neighbor": "AquaticAve.:WaterfoulWay", "distance": 158},
            {"neighbor": "MigrationAve.:WaddleWay", "distance": 124},
            {"neighbor": "PondsideAve.:WaterfoulWay", "distance": 160},
            {"neighbor": "MigrationAve.:FeatherSt.", "distance": 138}
        ]
    },
    {
        "name": "PondsideAve.:WaterfoulWay",
        "x": 214,
        "y": 241,
        "connections": [
            {"neighbor": "MigrationAve.:WaterfoulWay", "distance": 160},
            {"neighbor": "PondsideAve.:WaddleWay", "distance": 95},
            {"neighbor": "TheCircle:WaterfoulWay", "distance": 174},
            {"neighbor": "PondsideAve.:FeatherSt.", "distance": 136}
        ]
    },
    {
        "name": "TheCircle:WaterfoulWay",
        "x": 273,
        "y": 307,
        "connections": [
            {"neighbor": "PondsideAve.:WaterfoulWay", "distance": 174},
            {"neighbor": "TheCircle:FeatherSt.", "distance": 55},
            {"neighbor": "BreadcrumbAve.:TheCircle", "distance": 179}
        ]
    },
    {
        "name": "AquaticAve.:FeatherSt.",
        "x": 305,
        "y": 29,
        "connections": [
            {"neighbor": "AquaticAve.:WaterfoulWay", "distance": 133},
            {"neighbor": "MigrationAve.:FeatherSt.", "distance": 161},
            {"neighbor": "AquaticAve.:BeckSt.", "distance": 218}
        ]
    },
    {
        "name": "MigrationAve.:FeatherSt.",
        "x": 305,
        "y": 135,
        "connections": [
            {"neighbor": "AquaticAve.:FeatherSt.", "distance": 161},
            {"neighbor": "MigrationAve.:WaterfoulWay", "distance": 138},
            {"neighbor": "PondsideAve.:FeatherSt.", "distance": 148},
            {"neighbor": "MigrationAve.:BeckSt.", "distance": 218}
        ]
    },
    {
        "name": "PondsideAve.:FeatherSt.",
        "x": 305,
        "y": 233,
        "connections": [
            {"neighbor": "MigrationAve.:FeatherSt.", "distance": 148},
            {"neighbor": "PondsideAve.:WaterfoulWay", "distance": 136},
            {"neighbor": "TheCircle:FeatherSt.", "distance": 95},
            {"neighbor": "PondsideAve.:BeckSt.", "distance": 219}
        ]
    },
    {
        "name": "TheCircle:FeatherSt.",
        "x": 305,
        "y": 296,
        "connections": [
            {"neighbor": "PondsideAve.:FeatherSt.", "distance": 95},
            {"neighbor": "TheCircle:WaterfoulWay", "distance": 55},
            {"neighbor": "DabblerDr.:TheCircle", "distance": 96}
        ]
    },
    {
        "name": "AquaticAve.:BeckSt.",
        "x": 452,
        "y": 29,
        "connections": [
            {"neighbor": "AquaticAve.:FeatherSt.", "distance": 218},
            {"neighbor": "MigrationAve.:BeckSt.", "distance": 156},
            {"neighbor": "MigrationAve.:MallardSt.", "distance": 343}
        ]
    },
    {
        "name": "MigrationAve.:BeckSt.",
        "x": 452,
        "y": 135,
        "connections": [
            {"neighbor": "AquaticAve.:BeckSt.", "distance": 156},
            {"neighbor": "MigrationAve.:FeatherSt.", "distance": 218},
            {"neighbor": "PondsideAve.:BeckSt.", "distance": 149},
            {"neighbor": "MigrationAve.:MallardSt.", "distance": 194}
        ]
    },
    {
        "name": "PondsideAve.:BeckSt.",
        "x": 452,
        "y": 233,
        "connections": [
            {"neighbor": "MigrationAve.:BeckSt.", "distance": 149},
            {"neighbor": "PondsideAve.:FeatherSt.", "distance": 219},
            {"neighbor": "DabblerDr.:BeckSt.", "distance": 89},
            {"neighbor": "PondsideAve.:MallardSt.", "distance": 197}
        ]
    },
    {
        "name": "DabblerDr.:BeckSt.",
        "x": 452,
        "y": 293,
        "connections": [
            {"neighbor": "PondsideAve.:BeckSt.", "distance": 89},
            {"neighbor": "DabblerDr.:TheCircle", "distance": 173},
            {"neighbor": "DrakeDr.:BeckSt.", "distance": 160},
            {"neighbor": "DabblerDr.:MallardSt.", "distance": 192}
        ]
    },
    {
        "name": "DrakeDr.:BeckSt.",
        "x": 452,
        "y": 402,
        "connections": [
            {"neighbor": "DabblerDr.:BeckSt.", "distance": 160},
            {"neighbor": "DucklingDr.:BeckSt.", "distance": 92},
            {"neighbor": "DrakeDr.:MallardSt.", "distance": 250}
        ]
    },
    {
        "name": "DucklingDr.:BeckSt.",
        "x": 452,
        "y": 474,
        "connections": [
            {"neighbor": "DrakeDr.:BeckSt.", "distance": 92},
            {"neighbor": "TailAve.:BeckSt.", "distance": 15},
            {"neighbor": "DucklingDr.:MallardSt.", "distance": 371}
        ]
    },
    {
        "name": "TailAve.:BeckSt.",
        "x": 452,
        "y": 465,
        "connections": [
            {"neighbor": "DucklingDr.:BeckSt.", "distance": 15},
            {"neighbor": "TailAve.:TheCircle", "distance": 227}
        ]
    },
    {
        "name": "MigrationAve.:MallardSt.",
        "x": 585,
        "y": 135,
        "connections": [
            {"neighbor": "AquaticAve.:BeckSt.", "distance": 343},
            {"neighbor": "MigrationAve.:BeckSt.", "distance": 194},
            {"neighbor": "PondsideAve.:MallardSt.", "distance": 145}
        ]
    },
    {
        "name": "PondsideAve.:MallardSt.",
        "x": 585,
        "y": 233,
        "connections": [
            {"neighbor": "MigrationAve.:MallardSt.", "distance": 145},
            {"neighbor": "PondsideAve.:BeckSt.", "distance": 197},
            {"neighbor": "DabblerDr.:MallardSt.", "distance": 89}
        ]
    },
    {
        "name": "DabblerDr.:MallardSt.",
        "x": 585,
        "y": 293,
        "connections": [
            {"neighbor": "PondsideAve.:MallardSt.", "distance": 89},
            {"neighbor": "DabblerDr.:BeckSt.", "distance": 192},
            {"neighbor": "DrakeDr.:MallardSt.", "distance": 100},
            {"neighbor": "DucklingDr.:MallardSt.", "distance": 96}
        ]
    },
    {
        "name": "DrakeDr.:MallardSt.",
        "x": 576,
        "y": 354,
        "connections": [
            {"neighbor": "DabblerDr.:MallardSt.", "distance": 100},
            {"neighbor": "DrakeDr.:BeckSt.", "distance": 250}
        ]
    },
    {
        "name": "DucklingDr.:MallardSt.",
        "x": 593,
        "y": 354,
        "connections": [
            {"neighbor": "DabblerDr.:MallardSt.", "distance": 96},
            {"neighbor": "DucklingDr.:BeckSt.", "distance": 371}
        ]
    },
    {
        "name": "BreadcrumbAve.:TheCircle",
        "x": 284,
        "y": 393,
        "connections": [
            {"neighbor": "TheCircle:WaterfoulWay", "distance": 179},
            {"neighbor": "BreadcrumbAve.:WaddleWay", "distance": 214},
            {"neighbor": "TailAve.:TheCircle", "distance": 92}
        ]
    },
    {
        "name": "TailAve.:TheCircle",
        "x": 335,
        "y": 387,
        "connections": [
            {"neighbor": "DabblerDr.:TheCircle", "distance": 109},
            {"neighbor": "BreadcrumbAve.:TheCircle", "distance": 92},
            {"neighbor": "TailAve.:BeckSt.", "distance": 277}
        ]
    },
    {
        "name": "DabblerDr.:TheCircle",
        "x": 350,
        "y": 324,
        "connections": [
            {"neighbor": "TheCircle:FeatherSt.", "distance": 96},
            {"neighbor": "TailAve.:TheCircle", "distance": 109},
            {"neighbor": "DabblerDr.:BeckSt.", "distance": 173}
        ]
    },
]

# contains sequence of movements from one intersection to another
class State(Enum):
  Wait=0
  Forward=1
  TurnR=2
  TurnL=3
  RoundAbout=4
  Park=5

# List of actions to travel from current intersection to next intersection is retrieved through 
# PATH_DATA[current intersection]["paths"][(previous intersection, next intersection)]
PATH_DATA = {
    "PondsideAve.:QuackSt":{
        "paths": {
            ("MigrationAve.:QuackSt.", "BreadcrumbAve.:WaddleWay") : [["straight", 150], ["turn_right", 90], ["straight", 82]], 
            ("MigrationAve.:QuackSt.", "PondsideAve.:WaddleWay") : [["turn_right", 116], ["straight", 54]],
            ("BreadcrumbAve.:WaddleWay", "PondsideAve.:WaddleWay") : [["turn_left", 64], ["straight", 102]],
            ("BreadcrumbAve.:WaddleWay", "MigrationAve.:QuackSt.") : [["straight", 184]], 
            ("PondsideAve.:WaddleWay", "MigrationAve.:QuackSt.") : [["turn_left", 116], ["straight", 138]],
            ("PondsideAve.:WaddleWay", "BreadcrumbAve.:WaddleWay") : [["turn_right", 64], ["straight", 65], ["turn_right", 90], ["straight", 82]],
        }
    },
    "MigrationAve.:QuackSt.": {
        "paths": {
            ("PondsideAve.:QuackSt", "AquaticAve.:WaddleWay") : [["straight", 88], ["turn_left", 38], ["straight", 115], ["turn_left", 52], ["straight", 31]],
            ("PondsideAve.:QuackSt", "MigrationAve.:WaddleWay") : [["turn_left", 90], ["straight", 49]],
            ("MigrationAve.:WaddleWay", "AquaticAve.:WaddleWay") : [["straight", 18], ["turn_left", 90], ["straight", 46], ["turn_left", 38], ["straight", 115], ["turn_left", 52], ["straight", 31]],

            ("AquaticAve.:WaddleWay", "PondsideAve.:QuackSt") : [["straight", 171]],
            ("MigrationAve.:WaddleWay", "PondsideAve.:QuackSt") : [["turn_right", 90], ["straight", 91]],
            ("AquaticAve.:WaddleWay", "MigrationAve.:WaddleWay") : [["turn_right", 90], ["straight", 31]]
        }
    },
    "AquaticAve.:WaddleWay": { 
        "paths": {
            ("MigrationAve.:QuackSt.", "AquaticAve.:WaterfoulWay") : [["straight", 87]],
            ("MigrationAve.:WaddleWay", "AquaticAve.:WaterfoulWay") : [["straight", 22], ["turn_left", 90], ["straight", 25]],
            ("MigrationAve.:WaddleWay", "MigrationAve.:QuackSt.") : [["turn_right", 90], ["straight", 31], ["turn_right", 90], ["straight", 30]], 
            ("AquaticAve.:WaterfoulWay", "MigrationAve.:QuackSt.") : [["straight", 86], ["turn_right", 90], ["straight", 30]],
        }
    },
    "MigrationAve.:WaddleWay": { 
        "paths": {
            ("MigrationAve.:QuackSt.", "MigrationAve.:WaterfoulWay") : [["straight", 85]],
            ("PondsideAve.:WaddleWay", "MigrationAve.:WaterfoulWay") : [["straight", 20], ["turn_left", 90], ["straight", 26]],
            ("MigrationAve.:QuackSt.", "AquaticAve.:WaddleWay") : [["turn_right", 90], ["straight", 26]], 
            ("PondsideAve.:WaddleWay", "AquaticAve.:WaddleWay") : [["straight", 107]],
            ("MigrationAve.:WaterfoulWay", "AquaticAve.:WaddleWay") : [["turn_left", 90], ["straight", 19]],
            ("PondsideAve.:WaddleWay", "MigrationAve.:QuackSt.") : [["turn_right", 90], ["straight", 31]],
            ("MigrationAve.:WaterfoulWay", "MigrationAve.:QuackSt.") : [["straight", 91]]
        }
    },
    "PondsideAve.:WaddleWay": {
        "paths": {
            ("PondsideAve.:QuackSt", "MigrationAve.:WaddleWay") : [["turn_right", 90], ["straight", 47], ["turn_left", 17]],
            ("PondsideAve.:QuackSt", "PondsideAve.:WaterfoulWay") : [["straight", 56]],
            ("BreadcrumbAve.:WaddleWay", "PondsideAve.:WaterfoulWay") : [["turn_right", 30], ["turn_left", 100]],
            ("BreadcrumbAve.:WaddleWay", "MigrationAve.:WaddleWay") : [["straight", 124], ["turn_left", 17]],
            ("BreadcrumbAve.:WaddleWay", "PondsideAve.:QuackSt") : [["turn_right", 90], ["straight", 80]],
            ("PondsideAve.:WaterfoulWay", "MigrationAve.:WaddleWay") : [["turn_left", 90], ["straight", 54], ["turn_left", 17]],
            ("PondsideAve.:WaterfoulWay", "PondsideAve.:QuackSt") : [["straight", 142]],
        }
    },
    "BreadcrumbAve.:WaddleWay": {
        "paths":{
            ("PondsideAve.:QuackSt", "BreadcrumbAve.:TheCircle") : [["straight", 86], ["turn_right", 45], ["straight", 23]],
            ("PondsideAve.:QuackSt", "PondsideAve.:WaddleWay") : [["turn_right", 90], ["straight", 66], []],
            ("BreadcrumbAve.:TheCircle", "PondsideAve.:WaddleWay") : [["turn_left", 90], ["straight", 89], ["turn_right", 17], ["straight", 38]],
            ("BreadcrumbAve.:TheCircle", "PondsideAve.:QuackSt") : [["straight", 141], ["turn_left", 45], ["straight", 33], ["turn_left", 45], ["straight", 71]]
        }
    },
    "AquaticAve.:WaterfoulWay": {
        "paths": {
            ("AquaticAve.:WaddleWay", "MigrationAve.:WaterfoulWay") : [["turn_left", 90], ["straight", 45]],
            ("AquaticAve.:WaddleWay", "AquaticAve.:FeatherSt.") : [["straight", 99]], 
            ("AquaticAve.:FeatherSt.", "AquaticAve.:WaddleWay") : [["straight", 92]],
            ("AquaticAve.:FeatherSt.", "MigrationAve.:WaterfoulWay") : [["turn_right", 90], ["straight", 30]],
        }
    },
    "MigrationAve.:WaterfoulWay": { 
        "paths": {
            ("AquaticAve.:WaterfoulWay", "MigrationAve.:WaddleWay") : [["straight", 19], ["turn_left", 90], ["straight", 26]],
            ("AquaticAve.:WaterfoulWay", "PondsideAve.:WaterfoulWay") : [["straight", 99]],
            ("AquaticAve.:WaterfoulWay", "MigrationAve.:FeatherSt.") : [["turn_right", 90], ["straight", 20]],
            ("MigrationAve.:WaddleWay", "PondsideAve.:WaterfoulWay") : [["turn_left", 90], ["straight", 38]],
            ("MigrationAve.:WaddleWay", "MigrationAve.:FeatherSt.") : [["straight", 82]],
            ("MigrationAve.:FeatherSt.", "MigrationAve.:WaddleWay") : [["straight", 88]],
            ("MigrationAve.:FeatherSt.", "PondsideAve.:WaterfoulWay") : [["turn_right", 90], ["straight", 16]],
        }
    },
    "PondsideAve.:WaterfoulWay": { # Jim
        "paths": {
            ("MigrationAve.:WaterfoulWay", "PondsideAve.:WaddleWay") : [["straight", 35], ["turn_left", 60], ["straight", 16]],
            ("MigrationAve.:WaterfoulWay", "TheCircle:WaterfoulWay") : [["straight", 77], ["turn_right", 60], ["straight", 7]],
            ("MigrationAve.:WaterfoulWay", "PondsideAve.:FeatherSt.") : [["turn_right", 90], ["straight", 17]],
            ("PondsideAve.:WaddleWay", "TheCircle:WaterfoulWay") : [["turn_left", 90], ["turn_right", 60],  ["straight", 7]],
            ("PondsideAve.:WaddleWay", "PondsideAve.:FeatherSt.") : [["straight", 62], ["turn_left", 30]],
            ("PondsideAve.:FeatherSt.", "PondsideAve.:WaddleWay") : [["straight", 61]],
            ("PondsideAve.:FeatherSt.", "TheCircle:WaterfoulWay") : [["turn_right", 140]]
        }
    },
    "TheCircle:WaterfoulWay": {
        "paths": {
            ("PondsideAve.:WaterfoulWay", "TheCircle:FeatherSt.") : [],
            ("PondsideAve.:WaterfoulWay", "BreadcrumbAve.:TheCircle") : [],
            ("TheCircle:FeatherSt.", "BreadcrumbAve.:TheCircle") : []
        }
    },
    "AquaticAve.:FeatherSt.": {
        "paths": {
            ("AquaticAve.:WaterfoulWay", "MigrationAve.:FeatherSt.") : [["straight", 15],["turn_left", -60], ["straight", 14]],
            ("AquaticAve.:WaterfoulWay", "AquaticAve.:BeckSt.") : [["straight", 84]],
            ("MigrationAve.:FeatherSt.", "AquaticAve.:BeckSt.") : [["straight", 15],["turn_left", -60], ["straight", 45]]
        }
    },
    "MigrationAve.:FeatherSt.": {
        "paths": {
            ("AquaticAve.:FeatherSt.", "MigrationAve.:WaterfoulWay") : [["straight", 15],["turn_left", -60], ["straight", 14]],
            ("AquaticAve.:FeatherSt.", "PondsideAve.:FeatherSt.") : [["straight", 54.5]],
            ("AquaticAve.:FeatherSt.", "MigrationAve.:BeckSt.") : [["turn_right", 60], ["straight", 27.5], "STOP SIGN"],
            ("MigrationAve.:WaterfoulWay", "PondsideAve.:FeatherSt.") : [["straight", 15],["turn_left", -60], ["straight", 15]],
            ("MigrationAve.:WaterfoulWay", "MigrationAve.:BeckSt.") : [["straight", 84]],
            ("PondsideAve.:FeatherSt.", "MigrationAve.:BeckSt.") : [["straight", 15],["turn_left", -60], ["straight", 38.5]]
        }
    },
    "PondsideAve.:FeatherSt.": {
        "paths": {
            ("MigrationAve.:FeatherSt.", "PondsideAve.:WaterfoulWay") : [["straight", 15],["turn_left", -60], ["straight", 25.5]],
            ("MigrationAve.:FeatherSt.", "TheCircle:FeatherSt.") : [["straight", 46.5]],
            ("MigrationAve.:FeatherSt.", "PondsideAve.:BeckSt." ) : [["turn_right", 60], ["straight", 27.5], "STOP SIGN"],
            ("PondsideAve.:WaterfoulWay", "TheCircle:FeatherSt.") : ["STOP SIGN", ["straight", 15],["turn_left", -60]],
            ("PondsideAve.:WaterfoulWay", "PondsideAve.:BeckSt.") : ["STOP SIGN", ["straight", 84]],
            ("TheCircle:FeatherSt.", "PondsideAve.:BeckSt.") : [["straight", 15], ["turn_left", -60],["straight", 38.5]]
        }
    },
    "TheCircle:FeatherSt.": {
        "paths": {
            ("PondsideAve.:FeatherSt.", "TheCircle:WaterfoulWay") : [],
            ("PondsideAve.:FeatherSt.", "DabblerDr.:TheCircle") : [],
            ("TheCircle:WaterfoulWay", "DabblerDr.:TheCircle") : [],
        }
    },
    "AquaticAve.:BeckSt.": {
        "paths": {
            ("AquaticAve.:FeatherSt.", "MigrationAve.:BeckSt.") : [["straight", 15],["turn_left", -60], ["straight", 15]],
            ("AquaticAve.:FeatherSt.", "MigrationAve.:MallardSt.") : [["straight", 75], ["turn_left", 60], ["straight", 15]],
            ("MigrationAve.:BeckSt.", "MigrationAve.:MallardSt.") : [["straight", 79]],
        }
    },
    "MigrationAve.:BeckSt.": {
        "paths": {
            ("AquaticAve.:BeckSt.", "MigrationAve.:FeatherSt.") : [["straight", 15],["turn_left", -60], ["straight", 38]],
            ("AquaticAve.:BeckSt.", "PondsideAve.:BeckSt.") : [["straight", 60]],
            ("AquaticAve.:BeckSt.", "MigrationAve.:MallardSt.") : [["turn_right", 60], ["straight", 17]],
            ("MigrationAve.:FeatherSt.", "PondsideAve.:BeckSt.") : [["straight", 15],["turn_left", -60], ["straight", 15]],
            ("MigrationAve.:FeatherSt.", "MigrationAve.:MallardSt.") : [["straight", 79]],
            ("PondsideAve.:BeckSt.", "MigrationAve.:MallardSt.") : [["straight", 15],["turn_left", -60], ["straight", 34]]
        }
    },
    "PondsideAve.:BeckSt.": {
        "paths": {
            ("MigrationAve.:BeckSt.", "PondsideAve.:FeatherSt.") : [["straight", 15],["turn_left", -60], ["straight", 38]],
            ("MigrationAve.:BeckSt.", "DabblerDr.:BeckSt.") : [["straight", 43]],
            ("MigrationAve.:BeckSt.", "PondsideAve.:MallardSt.") : [["turn_right", 60], ["straight", 17]],
            ("PondsideAve.:FeatherSt.", "DabblerDr.:BeckSt.") : [["straight", 15], ["turn_left", 60]],
            ("PondsideAve.:FeatherSt.", "PondsideAve.:MallardSt.") : [["straight", 79]],
            ("DabblerDr.:BeckSt.", "PondsideAve.:MallardSt.") : [["straight", 15],["turn_left", -60], ["straight", 43.5]]
        }
    },
    "DabblerDr.:BeckSt.": {
        "paths": {
            ("PondsideAve.:BeckSt.", "DabblerDr.:TheCircle") : [],
            ("PondsideAve.:BeckSt.", "DrakeDr.:BeckSt.") : [],
            ("PondsideAve.:BeckSt.", "DabblerDr.:MallardSt.") : [],
            ("DabblerDr.:TheCircle", "DrakeDr.:BeckSt.") : [],
            ("DabblerDr.:TheCircle", "DabblerDr.:MallardSt.") : [],
            ("DrakeDr.:BeckSt.", "DabblerDr.:MallardSt.") : []
        }
    },
    "DrakeDr.:BeckSt.": {
        "paths": {
            ("DabblerDr.:BeckSt.", "DucklingDr.:BeckSt.") : [],
            ("DabblerDr.:BeckSt.", "DrakeDr.:MallardSt.") : [],
            ("DucklingDr.:BeckSt.", "DrakeDr.:MallardSt.") : [],
        }
    },
    "DucklingDr.:BeckSt.": {
        "paths": {
            ("DrakeDr.:BeckSt.", "TailAve.:BeckSt.") : [],
            ("DrakeDr.:BeckSt.", "DucklingDr.:MallardSt.") : [],
            ("TailAve.:BeckSt.", "DucklingDr.:MallardSt.") : []
        }
    },
    "TailAve.:BeckSt.": {
        "paths": {
            ("DucklingDr.:BeckSt.", "TailAve.:TheCircle") : []
        }
    },
    "MigrationAve.:MallardSt.": {
        "paths": {
            ("AquaticAve.:BeckSt.", "MigrationAve.:BeckSt.") : [["straight", 15],["turn_left", -60], ["straight", 30]],
            ("AquaticAve.:BeckSt.", "PondsideAve.:MallardSt.") : [["straight", 60]],
            ("MigrationAve.:BeckSt.", "PondsideAve.:MallardSt.") : [["straight", 15],["turn_left", -60], ["straight", 32]]
        }
    },
    "PondsideAve.:MallardSt.": {
        "paths": {
            ("MigrationAve.:MallardSt.", "DabblerDr.:MallardSt.") : [["straight", 39]],
            ("MigrationAve.:MallardSt.", "PondsideAve.:BeckSt.") : [["straight", 15],["turn_left", -60], ["straight", 26.5]],
            ("PondsideAve.:BeckSt.", "DabblerDr.:MallardSt.") : [["straight", 15],["turn_left", -60], ["straight", 28.5]]
        }
    },
    "DabblerDr.:MallardSt.": {
        "paths": {
            ("PondsideAve.:MallardSt.", "DabblerDr.:BeckSt.") : [],
            ("PondsideAve.:MallardSt.", "DrakeDr.:MallardSt.") : [],
            ("PondsideAve.:MallardSt.", "DucklingDr.:MallardSt.") : [],
            ("DabblerDr.:BeckSt.", "DrakeDr.:MallardSt.") : [],
            ("DabblerDr.:BeckSt.", "DucklingDr.:MallardSt.") : [],
            ("DrakeDr.:MallardSt.", "DucklingDr.:MallardSt." ) : [],
        }
    },
    "DrakeDr.:MallardSt.": {
        "paths": {
            ("DabblerDr.:MallardSt.", "DrakeDr.:BeckSt.") : []
        }
    },
    "DucklingDr.:MallardSt.": {
        "paths": {
            ("DabblerDr.:MallardSt.", "DucklingDr.:BeckSt.") : []
        }
    },
    "BreadcrumbAve.:TheCircle": {
        "paths": {
            ("TheCircle:WaterfoulWay", "BreadcrumbAve.:WaddleWay") : [],
            ("TheCircle:WaterfoulWay","TailAve.:TheCircle") : [],
            ("BreadcrumbAve.:WaddleWay", "TailAve.:TheCircle") : []
        }
    },
    "TailAve.:TheCircle": {
        "paths": {
            ("DabblerDr.:TheCircle", "BreadcrumbAve.:TheCircle") : [],
            ("DabblerDr.:TheCircle", "TailAve.:BeckSt.") : [],
            ("BreadcrumbAve.:TheCircle", "TailAve.:BeckSt.") : []
        }
    },
    "DabblerDr.:TheCircle": {
        "paths": {
            ("TheCircle:FeatherSt.", "TailAve.:TheCircle") : [],
            ("TheCircle:FeatherSt.", "DabblerDr.:BeckSt.") : [],
            ("TailAve.:TheCircle", "DabblerDr.:BeckSt.") : []
        }
    },
}


NODE_COORDS = {}
MAP_GRAPH   = {}

for entry in NODES_DATA:
    node_name = entry["name"]
    x_coord   = entry["x"]
    y_coord   = entry["y"]
    NODE_COORDS[node_name] = (x_coord, y_coord)
    
    MAP_GRAPH[node_name] = []
    for conn in entry["connections"]:
        neighbor = conn["neighbor"]
        dist     = conn["distance"]
        MAP_GRAPH[node_name].append((neighbor, dist))


class Zone(Enum):
  NOZONE=0
  Pedestrian=1
  Risk=2
  School=3
  Residential=4
  Shops=5
  Industrial=6

# temp class while waiting for long term decision
class Node:
  def __init__(self):
    pass

CURRENT_ZONE=Zone.NOZONE
CURRENT_STATE=State.Wait
NEXT_STATE=State.Wait