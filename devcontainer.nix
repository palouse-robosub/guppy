{ pkgs }:
let
  user = "vscode";
  uid = "1000";
  gid = "1000";
in
pkgs.dockerTools.buildLayeredImage {
  name = "guppy-dev";
  tag = "latest";

  contents = with pkgs; [
    bashInteractive
    coreutils-full
    which
    gnugrep

    git
  ];
}