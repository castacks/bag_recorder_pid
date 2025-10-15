# Bag Recorder

A simple bag recording node. This allows for multi-bag recording, configurable topic recording, and most importantly, interrupt routines between Ctrl+C and script stopping.

## Deployment
```bash
python3 scripts/bagger.py -c config/example.yaml -o my_bag
```
For help writing a config, see [example.yaml](config/example.yaml).

See this [demo video](https://youtu.be/YKiL-O7EJEg). Note `atvbag` was just an alias around `python scripts.bagger.py ...`
[<img height="720" alt="image" src="docs/bag_recorder_demo.gif" />](https://youtu.be/YKiL-O7EJEg)

> [!NOTE]
> This implementation is originally based off of @VarCoder, @yaoyuh-cmu and @kabirkedia's work during their time in the DARPA Triage Challenge on [team Chiron](https://teamchiron.ai/), but **heavy** modifications have been made since then.
