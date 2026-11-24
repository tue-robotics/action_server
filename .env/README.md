# Local settings and keys

Files in this folder hold settings that differ per machine, including API keys. Only the `*.example` files and this
README are committed; everything else in the folder is ignored by git.

| File | For |
|---|---|
| `llm.env` | The LLM task planner (`action_server.llm`): service, model and API key. Copy `llm.env.example` and fill it in. |

Never paste a key anywhere else (code, launch files, chat, issues). If a key ends up in git, revoke it.

## LLM backends

The planner sends the same request to every backend: the robot's instructions
(`<robot>_bringup/parameters/llm/task_planner.md`, with the names from `robocup_knowledge` filled in), the command, and
a JSON schema the answer has to follow. Which backend gets it is decided by one setting, `LLM_PROVIDER`; the rest of
the planner does not change.

| `LLM_PROVIDER` | Service | How it is called | What you need to provide |
|---|---|---|---|
| `foundry` | Claude in Microsoft Foundry | Messages API over HTTPS (`requests`) | `LLM_MODEL` = deployment name, `ANTHROPIC_FOUNDRY_API_KEY`, and `ANTHROPIC_FOUNDRY_BASE_URL` (the "Target URI" in the portal) or `ANTHROPIC_FOUNDRY_RESOURCE` |
| `anthropic` | Claude API (api.anthropic.com) | Messages API over HTTPS (`requests`) | `LLM_MODEL` = model id, `ANTHROPIC_API_KEY` |
| `azure` | Azure OpenAI (GPT models) | OpenAI SDK, `pip install openai` | `LLM_MODEL` = deployment name, `AZURE_OPENAI_API_KEY`, `AZURE_OPENAI_ENDPOINT`, `OPENAI_API_VERSION` (2024-08-01-preview or newer) |
| `openai` | OpenAI, or any server with the OpenAI API (vLLM, Ollama, ...) | OpenAI SDK, `pip install openai` | `LLM_MODEL`, `OPENAI_API_KEY`; `OPENAI_BASE_URL` for another server than OpenAI |
| `claude_code` | Claude through the Claude Code CLI on your machine | Runs `claude -p` per request | No key: Claude Code must be installed and logged in. `LLM_MODEL` optional (`sonnet`, `haiku`, `opus` or a model id) |

How each one works:

- **foundry / anthropic** post one request to `/v1/messages` (see [claude_backend.py](../action_server/src/action_server/llm/claude_backend.py)).
  Both are the same API at another address. The Anthropic SDK is not used because it has no release for the Python 3.8
  of ROS Noetic with the Foundry client or structured output.
- **azure / openai** use the chat completions API of the OpenAI SDK (see
  [backend.py](../action_server/src/action_server/llm/backend.py)).
- **claude_code** starts the CLI with tools, settings, MCP servers and skills off, so it is a plain model call, and
  the instructions replace Claude Code's own system prompt (see
  [claude_code_backend.py](../action_server/src/action_server/llm/claude_code_backend.py)). Every request starts a
  process, which adds a few seconds, and the login is a personal one: use it for development, not on the robot.

All backends ask for structured output, so the model can only answer JSON that follows the schema. For a model or
server without it, set `LLM_RESPONSE_FORMAT=json_object`; the schema is then only an instruction and the planner checks
the answer itself.

Settings that apply to more than one backend:

| Setting | Default | For |
|---|---|---|
| `LLM_TIMEOUT` | 20 | Seconds to wait for an answer. Use 60 with `claude_code`. |
| `LLM_EFFORT` | the model's | `foundry`, `anthropic`, `claude_code`: how much the model reasons first. `low` is enough for commands. |
| `LLM_TEMPERATURE` | the model's | `azure`, `openai` only. Leave unset for reasoning models. |
| `LLM_RESPONSE_FORMAT` | `json_schema` | `json_object` for models without structured output. |
| `LLM_KNOWLEDGE_FILE` | `$ROBOT_BRINGUP_PATH/parameters/llm/task_planner.md` | Another instructions file. |

[llm.env.example](llm.env.example) lists every setting with a comment.

## Switching backends

The settings are read when the planner starts, from the process environment first and then from `.env/llm.env`.
After a switch, restart what uses the planner (the conversation engine, or the smoke test).

**For good:** change `LLM_PROVIDER` and `LLM_MODEL` in `.env/llm.env`. The keys of the other backends can stay in the
file; only those of the chosen provider are read.

**For one run or one shell:** set the variables in the environment; they overrule the file.

```bash
LLM_PROVIDER=claude_code LLM_MODEL=sonnet LLM_TIMEOUT=60 rosrun action_server llm_smoke_test.py "bring me a coke"

export LLM_PROVIDER=claude_code LLM_MODEL=sonnet LLM_TIMEOUT=60   # before roslaunch: the whole stack uses it
```

`LLM_MODEL` usually has to change with the provider: a Foundry or Azure deployment name means nothing to another
backend.

**One file per backend:** keep a complete file per backend in this folder and point at the one to use.

```bash
cd <your action_server checkout>/.env
cp llm.env llm.claude_code.env       # and set the provider, model and timeout in the copy
export ACTION_SERVER_LLM_ENV_FILE=$(pwd)/llm.claude_code.env
unset ACTION_SERVER_LLM_ENV_FILE     # back to llm.env
```

## Claude Code on your own machine

To plan commands with your own Claude account, without keys for Foundry, Azure or OpenAI:

1. Install Claude Code, run `claude` once and log in with `/login`. Check that `claude -p "say ok"` answers. An older
   installation may not know the options the backend uses; `claude update` fixes that.
2. In `.env/llm.env` set `LLM_PROVIDER=claude_code`, `LLM_MODEL=sonnet` and `LLM_TIMEOUT=60`.
3. Check it with `rosrun action_server llm_smoke_test.py --ping`.

What can go wrong:

- **`Cannot run claude`**: the process that runs the planner does not have `~/.local/bin` on its `PATH`, for example
  when it is started by a launcher or service. Set `CLAUDE_CODE_PATH` to the full path of `claude`.
- **`Not logged in · Please run /login`**: step 1.
- **Who pays**: requests count against your Claude account, about 5000 tokens per command. When `ANTHROPIC_API_KEY` is
  exported in your shell, the CLI uses that key instead of your login. A key that is only in `llm.env` is not passed
  on.

## Checking the settings

```bash
rosrun action_server llm_smoke_test.py --ping   # one tiny request: are the key, the address and the model right?
rosrun action_server llm_smoke_test.py          # plans a few commands; needs ROBOT_ENV
```

`--ping` prints the provider and model it uses and sends the smallest request there is. For Claude in Microsoft
Foundry it is this one, which you can also send with curl when you want to rule out our code:

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
