#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 16 08:36:29 2018

@author: root

"""
import setGPU
import os
import torch
from POPQORN.vanilla_rnn.utils.sample_data import sample_mnist_data
import argparse
from POPQORN.vanilla_rnn.utils.verify_bound import verifyMaximumEps
from deprnn.get_bound_mnist import RNN
from deprnn.train_esn_mnist import test
from deprnn.wesad import load_wesad


def sample_wesad_data(
    N,
    seq_len,
    device,
    num_labels=4,
    train=False,
    shuffle=True,
    rnn=None,
    x=None,
    y=None,
):
    train_data, val_data, test_data = load_wesad()
    data = train_data if train else test_data

    with torch.no_grad():
        if x is None and y is None:
            data_loader = torch.utils.data.DataLoader(
                data, batch_size=N, shuffle=shuffle
            )
            iterater = iter(data_loader)
            x, y = iterater.next()
        x, y = x.to(device), y.to(device)
        x = x.view(N, seq_len, -1)
        # num = N
        rand_label = (
            torch.randint(num_labels - 1, [N], dtype=torch.long, device=device) + 1
        )
        # range from 1 to num_labels-1
        target_label = torch.fmod(y + rand_label, num_labels)
        if not rnn is None:
            out = rnn(x)
            pred = out.argmax(dim=1)
            idx = pred == y
            # num = idx.sum()
            x = x[idx]
            y = y[idx]
            target_label = target_label[idx]
            print("remained fraction: %.4f" % idx.float().mean())

    return x, y, target_label


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compute Certified Bound for Vanilla RNNs"
    )
    parser.add_argument(
        "--hidden-size",
        default=256,
        type=int,
        metavar="HS",
        help="hidden layer size (default: 64)",
    )
    parser.add_argument("--time-step", default=100, type=int, metavar="TS")
    parser.add_argument(
        "--activation",
        default="tanh",
        type=str,
        metavar="a",
        help="nonlinearity used in the RNN, can be either tanh or relu (default: tanh)",
    )
    parser.add_argument(
        "--model",
        default="rnn",
        type=str,
        metavar="MN",
        help="the name of the pretrained model (default: rnn)",
    )

    parser.add_argument(
        "--cuda", action="store_true", help="whether to allow gpu for training"
    )
    parser.add_argument(
        "--cuda-idx",
        default=0,
        type=int,
        metavar="CI",
        help="the index of the gpu to use if allow gpu usage (default: 0)",
    )

    parser.add_argument(
        "--N",
        default=50,
        type=int,
        help="number of samples to compute bounds for (default: 50)",
    )
    parser.add_argument(
        "--p",
        default=2,
        type=int,
        help="p norm, if p > 100, we will deem p = infinity (default: 2)",
    )
    parser.add_argument(
        "--eps0",
        default=0.1,
        type=float,
        help="the start value to search for epsilon (default: 0.1)",
    )
    args = parser.parse_args()

    allow_gpu = True  # args.cuda

    if torch.cuda.is_available() and allow_gpu:
        device = "cuda"
        # device = torch.device('cuda:%s' % args.cuda_idx)
    else:
        device = torch.device("cpu")

    N = args.N  # number of samples to handle at a time.
    p = args.p  # p norm
    if p > 100:
        p = float("inf")

    eps0 = args.eps0
    input_size = 14
    hidden_size = args.hidden_size
    output_size = 4
    time_step = args.time_step
    activation = args.activation
    model_name = args.model
    work_dir = f"models/wesad/{model_name}/{model_name}_{hidden_size}_{activation}/"
    model_file = work_dir + model_name
    save_dir = work_dir + "%s_norm_bound/" % str(p)

    # load model

    with torch.no_grad():
        rnn = RNN(input_size, hidden_size, output_size, time_step, activation)
        rnn.load_state_dict(torch.load(model_file, map_location="cpu"))
        rnn.to(device)

        train_data, val_data, test_data = load_wesad()
        val_loader = torch.utils.data.DataLoader(val_data, batch_size=64, shuffle=False)
        test(rnn, device, val_loader)
        # exit(0)

        X, y, target_label = sample_wesad_data(
            N,
            time_step,
            device,
            num_labels=4,
            train=False,
            shuffle=True,
            rnn=rnn,
            x=None,
            y=None,
        )

        rnn.extractWeight(clear_original_model=False)

        l_eps, u_eps = rnn.getMaximumEps(
            p=p,
            true_label=y,
            target_label=target_label,
            eps0=eps0,
            max_iter=100,
            X=X,
            acc=1e-3,
            gx0_trick=True,
            Eps_idx=None,
        )
        verifyMaximumEps(
            rnn,
            X,
            l_eps,
            p,
            y,
            target_label,
            eps_idx=None,
            untargeted=False,
            thred=1e-8,
        )

        os.makedirs(save_dir, exist_ok=True)
        torch.save(
            {
                "l_eps": l_eps,
                "u_eps": u_eps,
                "X": X,
                "true_label": y,
                "target_label": target_label,
            },
            save_dir + "certified_bound",
        )
        print("Have saved the complete result to" + save_dir + "certified_bound")
        print("statistics of l_eps:")
        print(
            "(min, mean, max, std) = (%.4f, %.4f, %.4f, %.4f) "
            % (l_eps.min(), l_eps.mean(), l_eps.max(), l_eps.std())
        )
