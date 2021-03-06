#include <stdio.h>
#include <limits.h>
#include <stdlib.h>
#include <inttypes.h>
#include <command.h>
#include <log/log.h>
#include <conf_log.h>

LOG_GROUP_VERBOSE(user_group, "user");

int cmd_log_mask(struct command_context * ctx, int type)
{
	int mask_old, mask_new;
	char *token;

	if (ctx->argc != 3)
		return CMD_ERROR_SYNTAX;

	token = strtok(ctx->argv[1], ",");
	while (token != NULL) {

		if (strncasecmp(token, "all", strlen(token))) {
			mask_old = log_get_mask(token, type);
			if (mask_old < 0)
				return CMD_ERROR_SYNTAX;
		} else {
			mask_old = 0;
		}

		mask_new = log_string_to_mask(ctx->argv[2], mask_old);
		if (mask_new < 0)
			return CMD_ERROR_SYNTAX;

		printf("Set mask of group %s from 0x%02X to 0x%02X\r\n",
		       token, (uint8_t)mask_old, (uint8_t)mask_new);

		log_set_mask(token, (uint8_t)mask_new, type);

		token = strtok(NULL, ",");
	}

	return CMD_ERROR_NONE;
}

int cmd_log_printmask(struct command_context * ctx) {
	return cmd_log_mask(ctx, 1);
}

int cmd_log_storemask(struct command_context * ctx) {
	return cmd_log_mask(ctx, 0);
}

int cmd_log_list(struct command_context * ctx) {
	log_print_groups();
	return CMD_ERROR_NONE;
}

#ifdef ENABLE_LOG_STORE
int cmd_log_hist(struct command_context * ctx) {

	if (ctx->argc > 2)
		return CMD_ERROR_SYNTAX;

	static unsigned int count = 20;
	if (ctx->argc == 2) {
		count = atoi(ctx->argv[1]);
	}

	int iter(char * data, unsigned int size) {

		if (size < sizeof(log_store_t))
			return -1;

		log_store_t * store = (void *) data;
		if (store == NULL)
			return -1;

		if (store->len != size - offsetof(log_store_t, data))
			return -1;

		if (count-- == 0)
			return -1;

		/* Start color */
		switch (store->level) {
			case LOG_TRACE: printf("\E[0;35m"); break;	/* Magenta */
			case LOG_DEBUG: printf("\E[0;34m"); break;	/* Blue */
			case LOG_INFO: printf("\E[0;32m"); break;	/* Green */
			case LOG_WARNING: printf("\E[0;33m"); break;	/* Yellow */
			case LOG_ERROR: printf("\E[1;31m"); break;	/* Red */
			default: return -1; break;
		}

		/* Print time and group name */
		printf("%04"PRIu32".%06"PRIu32" %.10s: ", store->sec, store->nsec / 1000, store->group->name);

		/* Print log message */
#ifdef __AVR__
		printf("%s", store->data);
#else
		printf("%.*s", store->len, store->data);
#endif

		/* End color */
		printf("\E[0m\r\n");

		return 0;
	}

	log_store_foreach(iter);

	return CMD_ERROR_NONE;
}
#endif

int cmd_log_insert(struct command_context * ctx)
{
	log_level_t level;

	if (ctx->argc < 3)
		return CMD_ERROR_SYNTAX;

	if (!strcmp(ctx->argv[1], "trace"))
		level = LOG_TRACE;
	else if (!strcmp(ctx->argv[1], "debug"))
		level = LOG_DEBUG;
	else if (!strcmp(ctx->argv[1], "info"))
		level = LOG_INFO;
	else if (!strcmp(ctx->argv[1], "warn"))
		level = LOG_WARNING;
	else if (!strcmp(ctx->argv[1], "error"))
		level = LOG_ERROR;
	else
		return CMD_ERROR_SYNTAX;

	log_event_group(level, user_group, "%s", ctx->argv[2]);

	return CMD_ERROR_NONE;
}

int cmd_log_debug(struct command_context * ctx)
{
	return cmd_log_mask(ctx, 1);
}

command_t __sub_command cmd_logs[] = {
	{
		.name = "print",
		.help = "Set printmask",
		.usage = "<group>[,group] <Err|Wrn|Inf|Dbg|Trc|Std|All|No>",
		.handler = cmd_log_printmask,
	},{
		.name = "store",
		.help = "Set storemask",
		.usage = "<group>[,group] <Err|Wrn|Inf|Dbg|Trc|Std|All|No>",
		.handler = cmd_log_storemask,
	},{
		.name = "list",
		.help = "Show groups",
		.handler = cmd_log_list,
	},{
		.name = "insert",
		.help = "Insert log message",
		.usage = "<level> <message>",
		.handler = cmd_log_insert,
	},
#ifdef ENABLE_LOG_STORE
	{
		.name = "hist",
		.usage = "<cnt>",
		.help = "Show history",
		.handler = cmd_log_hist,
	}
#endif
};

command_t __root_command cmd_log[] = {
	{
		.name = "log",
		.help = "log: Log system",
		.chain = INIT_CHAIN(cmd_logs)
	},{
		.name = "debug",
		.help = "log: Alias of 'log print'",
		.usage = "<group> <Err|Wrn|Inf|Dbg|Trc|Std|All|No>",
		.handler = cmd_log_debug,
	},
};
