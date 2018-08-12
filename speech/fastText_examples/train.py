import fastText

file = raw_input("Enter file name to train...")
model = fastText.train_supervised(file)
model.save_model("robocup_actions.bin")
