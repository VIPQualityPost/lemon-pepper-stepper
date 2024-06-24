import pandas as pd
import os, sys, csv

def footprintFix(fpName):
    propertiesList = fpName.split('_')

    if len(propertiesList) == 4:
        if  any(rcl in propertiesList[0] for rcl in ["Resistor", "Capacitor", "Inductor", "Diode"]):
            # For things like R/C/L, return something like C0402
            return propertiesList[1][-1] + propertiesList[2]

    # Strip out library name from FP.
    return fpName.split(":")[-1]

bom_file = pd.read_csv(sys.argv[1])

bom_file.columns = ['Designator', 'Value', 'Footprint', 'JLCPCB Part #']
bom_file['Footprint'] = bom_file['Footprint'].apply(lambda x: footprintFix(x))

bom_file.to_csv(sys.argv[2], index=False)
