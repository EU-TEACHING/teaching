#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Dec  2 10:30:14 2018

@author: root
This files helps train a RNN
"""
import setGPU
import torch
import POPQORN.vanilla_rnn.bound_vanilla_rnn as v_rnn
import torch.optim as optim
import argparse
import os

from deprnn.wesad import load_wesad
from deprnn.utils import train, test
import numpy as np


def main(model, savefile, cuda):
    batch_size = 64
    test_batch_size = 128
    epochs = 200
    log_interval = 10
    lr = 0.001
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
    val_loader = torch.utils.data.DataLoader(
        val_data, batch_size=test_batch_size, shuffle=shuffle_test, **kwargs
    )
    optimizer = optim.Adam(model.parameters(), lr=lr)

    best_acc = -1
    for epoch in range(1, epochs + 1):
        train(log_interval, model, device, train_loader, optimizer, epoch)
        curr_loss, curr_acc = test(model, device, val_loader)
        if curr_acc > best_acc:
            best_acc = curr_acc
            torch.save(model.cpu().state_dict(), savefile)
            print("have saved the trained model to ", savefile)

    test_loader = torch.utils.data.DataLoader(
        test_data, batch_size=test_batch_size, shuffle=shuffle_test, **kwargs
    )
    model.load_state_dict(torch.load(savefile))
    test(model, device, test_loader)
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train Mnist RNN Classifier")

    parser.add_argument(
        "--hidden-size",
        default=256,
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
        "--cuda", action="store_true", help="whether to allow gpu for training"
    )
    args = parser.parse_args()

    model = "rnn"
    save_dir = f"models/wesad/{model}/"
    input_size = 14
    hidden_size = args.hidden_size
    output_size = 4
    time_step = 100
    activation = args.activation

    if model == "rnn":
        rnn = v_rnn.RNN(input_size, hidden_size, output_size, time_step, activation)
    elif model == "esn":
        rnn = v_rnn.RNN(input_size, hidden_size, output_size, time_step, activation)
        # set rho
        with torch.no_grad():
            desired_rho = 0.9
            W = rnn.rnn.all_weights[0][1].data
            rho = np.max(np.abs(np.linalg.eigvals(W.numpy())))
            rnn.rnn.all_weights[0][1].data = W * desired_rho / rho
        for pars in rnn.rnn.parameters():
            pars.requires_grad = False
    else:
        assert False
    model_name = f"{model}_{hidden_size}_{activation}"
    save_dir = save_dir + model_name
    os.makedirs(save_dir, exist_ok=True)
    main(rnn, save_dir + f"/{model}", args.cuda)
