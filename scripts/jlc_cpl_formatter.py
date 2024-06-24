import pandas as pd
import os, sys, csv

cpl_file = pd.read_csv(sys.argv[1])

cpl_file.columns = ['Designator', 'Val', 'Package', 'Mid X', 'Mid Y', 'Rotation', 'Layer']
cpl_file = cpl_file.reindex(columns=['Designator', 'Mid X', 'Mid Y', 'Layer', 'Rotation'])
cpl_file.drop([0,1,2,3,4,5])
cpl_file['Layer'] = "T"

cpl_file.to_csv(sys.argv[2], index=False)
