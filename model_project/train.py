
from config import MODEL_TYPE
from train.train_lstm import train_lstm
from train.train_cnn import train_cnn
from train.train_svm import train_svm

def main():
    if MODEL_TYPE == "lstm":
        train_lstm()
    elif MODEL_TYPE == "cnn":
        train_cnn()
    elif MODEL_TYPE == "svm":
        train_svm()
    else:
        print("Unsupported model type.")

        

if __name__ == "__main__":
    main()
