import fastText

file = input("Enter file name to train...")
model = fastText.train_supervised(file)
model.save_model("safe_actions.bin")
