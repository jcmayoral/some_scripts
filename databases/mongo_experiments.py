from pymongo import MongoClient
import datetime

client = MongoClient('jose-HBRS', 27018)
db = client.experiments
collection = db['experiment']
print collection
print db
post = {"author": "Mike",
         "text": "My first blog post!",
         "tags": ["mongodb", "python", "pymongo"],
         "date": datetime.datetime.utcnow()}

posts = db.posts
post_id = posts.insert_one(post).inserted_id

