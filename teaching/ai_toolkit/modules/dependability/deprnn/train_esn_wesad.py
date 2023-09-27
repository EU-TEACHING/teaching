#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Dec  2 10:30:14 2018

@author: root
This files helps train a RNN
"""
import torch
import torch.nn as nn
import POPQORN.vanilla_rnn.bound_vanilla_rnn as v_rnn
from torchvision import datasets, transforms
import torch.optim as optim
import torch.nn.functional as F
import argparse
import os
from deprnn.wesad import load_wesad
from deprnn.utils import train, test


def main(model, savefile, cuda):
    batch_size = 64
    test_batch_size = 128
    epochs = 200
    log_interval = 10
    lr = 0.01
    momentum = 0.5
    shuffle_train = True
    shuffle_test = True

    use_cuda = cuda and torch.cuda.is_available()
    print("use_cuda: ", use_cuda)

    device = torch.device("cuda" if use_cuda else "cpu")
    model.to(device)
    kwargs = {"num_workers": 1, "pin_memory": True} if use_cuda else {}
    train_data, val_data, test_data = load_wesad()

    train_loader = torch.utils.data.DataLoader(
        train_data, batch_size=batch_size, shuffle=shuffle_train, **kwargs
    )
    test_loader = torch.utils.data.DataLoader(
        val_data, batch_size=test_batch_size, shuffle=shuffle_test, **kwargs
    )
    optimizer = optim.SGD(model.parameters(), lr=lr, momentum=momentum)
    for epoch in range(1, epochs + 1):
        train(log_interval, model, device, train_loader, optimizer, epoch)
        test(model, device, test_loader)

    torch.save(model.cpu().state_dict(), savefile)
    print("have saved the trained model to ", savefile)

    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train Mnist ESN Classifier")

    parser.add_argument(
        "--hidden-size",
        default=64,
        type=int,
        metavar="HS",
        help="hidden layer size (default: 64)",
    )
    parser.add_argument(
        "--activation",
        default="tanh",
        type=str,
        metavar="a",
        help="nonlinearity used in the RNN, can be either tanh or relu (default: tanh)",
    )
    parser.add_argument(
        "--save_dir",
        default="models/wesad/esn/",
        type=str,
        metavar="SD",
        help="the directory to save the trained model",
    )
    parser.add_argument(
        "--cuda", action="store_true", help="whether to allow gpu for training"
    )
    args = parser.parse_args()

    input_size = 14
    hidden_size = args.hidden_size
    output_size = 4
    time_step = 100
    activation = args.activation

    esn = v_rnn.RNN(input_size, hidden_size, output_size, time_step, activation)
    for pars in esn.rnn.parameters():
        pars.requires_grad = False

    model_name = "esn_%s_%s" % (str(hidden_size), activation)
    save_dir = args.save_dir + model_name
    os.makedirs(save_dir, exist_ok=True)
    main(esn, save_dir + "/esn", args.cuda)
