# Bag Recorder

A simple bag recording node. This allows for multi-bag recording, configurable topic recording, and most importantly, interrupt routines between Ctrl+C and script stopping.

## Deployment
```bash
python3 scripts/bagger.py -c config/example.yaml -o my_bag
```
For help writing a config, see [example.yaml](config/example.yaml).

> [!NOTE]
> This implementation is originally based off of @VarCoder, @yaoyuh-cmu and @kabirkedia's work during their time in the DARPA Triage Challenge on [team Chiron](https://teamchiron.ai/), but **heavy** modifications have been made since then.
