import json

log_file = "./Wolfgang/log.json"
new_log_file = "./Wolfgang/log_shortened.json"

with open(log_file, "r") as f:
    daten = json.load(f)
    kp = daten["kp"]
    entries = daten["entries"][:50000]

    daten = {
        "kp": kp, 
        "entries": entries
    }

    with open(new_log_file, "w") as f:
        json.dump(daten, f, indent=4)