from pymongo import MongoClient
import datetime

client = MongoClient('jose-HBRS', 27018)
db = client.experiments
collection = db['experiment']
print collection

