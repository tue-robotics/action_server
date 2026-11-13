# Local settings and keys

Files in this folder hold settings that differ per machine, including API keys. Only the `*.example` files and this
README are committed; everything else in the folder is ignored by git.

| File | For |
|---|---|
| `llm.env` | The LLM task planner (`action_server.llm`): service, model and API key. Copy `llm.env.example` and fill it in. |

Never paste a key anywhere else (code, launch files, chat, issues). If a key ends up in git, revoke it.

## Checking the settings

```bash
rosrun action_server llm_smoke_test.py --ping   # one tiny request: are the key, the address and the model right?
rosrun action_server llm_smoke_test.py          # plans a few commands; needs ROBOT_ENV
```

`--ping` sends the smallest request there is. For Claude in Microsoft Foundry it is this one, which you can also send
with curl when you want to rule out our code:

```bash
curl https://<resource>.services.ai.azure.com/anthropic/v1/messages \
  -H "x-api-key: $ANTHROPIC_FOUNDRY_API_KEY" \
  -H "anthropic-version: 2026-10-21" \
  -H "content-type: application/json" \
  -d '{"model": "<deployment>", "max_tokens": 64, "messages": [{"role": "user", "content": "Reply with the word ok."}]}'
```

A `401` means the key does not belong to this resource, or the header is wrong: our resource rejects the `api-key`
header that the Foundry documentation also mentions and accepts `x-api-key`. A `404` means the deployment name in
`LLM_MODEL` does not exist on the resource.
