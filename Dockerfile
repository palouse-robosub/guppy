FROM ubuntu:noble

RUN apt update
RUN apt install -y \
    nix \
    git \
    direnv

RUN mkdir -p /etc/nix && \
    echo "experimental-features = nix-command flakes" >> /etc/nix/nix.conf && \
    echo "extra-substituters = https://ros.cachix.org https://palouse-robosub.cachix.org" >> /etc/nix/nix.conf && \
    echo "extra-trusted-public-keys = ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo= palouse-robosub.cachix.org-1:r2KNmfNGOZB+IhqEqDIMDaEWMYZv8ct1tdSg7n7fNKw=" >> /etc/nix/nix.conf && \
    echo "accept-flake-config = true" >> /etc/nix/nix.conf

RUN echo 'eval "$(direnv hook bash)"' >> ~/.bashrc

RUN git config --global --add safe.directory /workspaces/guppy