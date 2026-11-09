// =============================================================================
// Manual Registration Snippet for `autogen/sl_cli_command_table.c`
// =============================================================================
// Note: If you import `velux.slcp`, Simplicity Studio will generate this entry
// automatically from the `template_contribution` section. If you start from a
// fresh `rail_soc_railtest` project, add these 3 snippets to
// `autogen/sl_cli_command_table.c`:

// 1. Add declaration near the top of `autogen/sl_cli_command_table.c`:
void cli_velux(sl_cli_command_arg_t *arguments);

// 2. Add command descriptor right above `sl_cli_default_command_table[]`:
static const sl_cli_command_info_t cli_cmd__velux = \
  SL_CLI_COMMAND(cli_velux,
                 "Control 2.4 GHz blind & skylight (open/close/stop/pos/status/pair/setseq/setkey/set_transfer_key/reset_nvm)",
                  "[blind|skylight] [open|close|stop|pos|status|pair]" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_STRINGOPT, SL_CLI_ARG_END, });

// 3. Add table entry at the end of `sl_cli_default_command_table[]` (right before `{ NULL, NULL, false }`):
  { "velux", &cli_cmd__velux, false },
