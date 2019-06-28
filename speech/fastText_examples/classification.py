import fastText
import string

m = fastText.load_model('safe_actions.bin')

for i in range(20):
    sentence = input("Enter sentence to query...")
    result = m.predict(sentence)
    action = str(result[0])
    action = action.replace("'", "")
    action = action.replace(",", "")
    action = action.replace("(", "")
    action = action.replace(")", "")
    result2 = m.predict(sentence,k=2)
    print ("Selected Action: ", action.replace("__label__", ""))
    print ("Selected Action: ", result2)
