# ness_check_bc.py

from pyke import contexts, pattern, bc_rule

pyke_version = '1.1.1'
compiler_version = 1

def trust_analysis(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('ness_fact', 'is_latest_status_list', context,
                          (rule.pattern(0),
                           rule.pattern(1),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "ness_check.trust_analysis: got unexpected plan from when clause 1"
            with engine.prove('ness_fact', 'is_server_status_list', context,
                              (rule.pattern(2),
                               rule.pattern(3),)) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "ness_check.trust_analysis: got unexpected plan from when clause 2"
                with engine.prove('ness_fact', 'is_flag_list', context,
                                  (rule.pattern(4),
                                   rule.pattern(5),)) \
                  as gen_3:
                  for x_3 in gen_3:
                    assert x_3 is None, \
                      "ness_check.trust_analysis: got unexpected plan from when clause 3"
                    with engine.prove('ness_fact', 'is_server_list', context,
                                      (rule.pattern(6),
                                       rule.pattern(7),)) \
                      as gen_4:
                      for x_4 in gen_4:
                        assert x_4 is None, \
                          "ness_check.trust_analysis: got unexpected plan from when clause 4"
                        with engine.prove('ness_fact', 'is_index', context,
                                          (rule.pattern(8),
                                           rule.pattern(9),)) \
                          as gen_5:
                          for x_5 in gen_5:
                            assert x_5 is None, \
                              "ness_check.trust_analysis: got unexpected plan from when clause 5"
                            with engine.prove('ness_fact', 'is_number_nodes', context,
                                              (rule.pattern(10),
                                               rule.pattern(11),)) \
                              as gen_6:
                              for x_6 in gen_6:
                                assert x_6 is None, \
                                  "ness_check.trust_analysis: got unexpected plan from when clause 6"
                                print("Inferred index = ", int(context.lookup_data('index')[0]))
                                print("Inferred nnode = ", int(context.lookup_data('nnode')[0]))
                                print("Inferred latest_status_list = ", context.lookup_data('latest_status_list'))
                                print("Inferred good_server_status_list = ", context.lookup_data('good_server_status_list'))
                                print("Inferred flags_list = ", context.lookup_data('flags_list'))
                                print("Inferred servers_list = ", context.lookup_data('servers_list'))
                                with engine.prove(rule.rule_base.root_name, 'check_range', context,
                                                  (rule.pattern(12),
                                                   rule.pattern(13),)) \
                                  as gen_8:
                                  for x_8 in gen_8:
                                    assert x_8 is None, \
                                      "ness_check.trust_analysis: got unexpected plan from when clause 8"
                                    with engine.prove(rule.rule_base.root_name, 'Table_Consistency', context,
                                                      (rule.pattern(14),
                                                       rule.pattern(15),
                                                       rule.pattern(16),)) \
                                      as gen_9:
                                      for x_9 in gen_9:
                                        assert x_9 is None, \
                                          "ness_check.trust_analysis: got unexpected plan from when clause 9"
                                        with engine.prove(rule.rule_base.root_name, 'Status_Analysis', context,
                                                          (rule.pattern(17),
                                                           rule.pattern(16),
                                                           rule.pattern(13),)) \
                                          as gen_10:
                                          for x_10 in gen_10:
                                            assert x_10 is None, \
                                              "ness_check.trust_analysis: got unexpected plan from when clause 10"
                                            rule.rule_base.num_bc_rule_successes += 1
                                            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def trust_analysis_1(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('ness_fact', 'is_latest_status_list', context,
                          (rule.pattern(0),
                           rule.pattern(1),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "ness_check.trust_analysis_1: got unexpected plan from when clause 1"
            with engine.prove('ness_fact', 'is_server_status_list', context,
                              (rule.pattern(2),
                               rule.pattern(3),)) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "ness_check.trust_analysis_1: got unexpected plan from when clause 2"
                with engine.prove('ness_fact', 'is_flag_list', context,
                                  (rule.pattern(4),
                                   rule.pattern(5),)) \
                  as gen_3:
                  for x_3 in gen_3:
                    assert x_3 is None, \
                      "ness_check.trust_analysis_1: got unexpected plan from when clause 3"
                    with engine.prove('ness_fact', 'is_server_list', context,
                                      (rule.pattern(6),
                                       rule.pattern(7),)) \
                      as gen_4:
                      for x_4 in gen_4:
                        assert x_4 is None, \
                          "ness_check.trust_analysis_1: got unexpected plan from when clause 4"
                        with engine.prove('ness_fact', 'is_index', context,
                                          (rule.pattern(8),
                                           rule.pattern(9),)) \
                          as gen_5:
                          for x_5 in gen_5:
                            assert x_5 is None, \
                              "ness_check.trust_analysis_1: got unexpected plan from when clause 5"
                            with engine.prove('ness_fact', 'is_number_nodes', context,
                                              (rule.pattern(10),
                                               rule.pattern(11),)) \
                              as gen_6:
                              for x_6 in gen_6:
                                assert x_6 is None, \
                                  "ness_check.trust_analysis_1: got unexpected plan from when clause 6"
                                with engine.prove(rule.rule_base.root_name, 'check_range', context,
                                                  (rule.pattern(12),
                                                   rule.pattern(13),)) \
                                  as gen_7:
                                  for x_7 in gen_7:
                                    assert x_7 is None, \
                                      "ness_check.trust_analysis_1: got unexpected plan from when clause 7"
                                    with engine.prove(rule.rule_base.root_name, 'Table_Consistency', context,
                                                      (rule.pattern(14),
                                                       rule.pattern(15),
                                                       rule.pattern(16),)) \
                                      as gen_8:
                                      for x_8 in gen_8:
                                        assert x_8 is None, \
                                          "ness_check.trust_analysis_1: got unexpected plan from when clause 8"
                                        with engine.prove(rule.rule_base.root_name, 'Status_Analysis_u', context,
                                                          (rule.pattern(17),
                                                           rule.pattern(16),
                                                           rule.pattern(13),)) \
                                          as gen_9:
                                          for x_9 in gen_9:
                                            assert x_9 is None, \
                                              "ness_check.trust_analysis_1: got unexpected plan from when clause 9"
                                            rule.rule_base.num_bc_rule_successes += 1
                                            yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def Consistent_data(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove('ness_fact', 'is_latest_status_list', context,
                          (rule.pattern(0),
                           rule.pattern(1),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "ness_check.Consistent_data: got unexpected plan from when clause 1"
            with engine.prove('ness_fact', 'is_server_status_list', context,
                              (rule.pattern(2),
                               rule.pattern(3),)) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "ness_check.Consistent_data: got unexpected plan from when clause 2"
                with engine.prove('ness_fact', 'is_flag_list', context,
                                  (rule.pattern(4),
                                   rule.pattern(5),)) \
                  as gen_3:
                  for x_3 in gen_3:
                    assert x_3 is None, \
                      "ness_check.Consistent_data: got unexpected plan from when clause 3"
                    with engine.prove('ness_fact', 'is_server_list', context,
                                      (rule.pattern(6),
                                       rule.pattern(7),)) \
                      as gen_4:
                      for x_4 in gen_4:
                        assert x_4 is None, \
                          "ness_check.Consistent_data: got unexpected plan from when clause 4"
                        with engine.prove('ness_fact', 'is_index', context,
                                          (rule.pattern(8),
                                           rule.pattern(9),)) \
                          as gen_5:
                          for x_5 in gen_5:
                            assert x_5 is None, \
                              "ness_check.Consistent_data: got unexpected plan from when clause 5"
                            with engine.prove('ness_fact', 'is_number_nodes', context,
                                              (rule.pattern(10),
                                               rule.pattern(11),)) \
                              as gen_6:
                              for x_6 in gen_6:
                                assert x_6 is None, \
                                  "ness_check.Consistent_data: got unexpected plan from when clause 6"
                                with engine.prove(rule.rule_base.root_name, 'check_range', context,
                                                  (rule.pattern(12),
                                                   rule.pattern(13),)) \
                                  as gen_7:
                                  for x_7 in gen_7:
                                    assert x_7 is None, \
                                      "ness_check.Consistent_data: got unexpected plan from when clause 7"
                                    with engine.prove(rule.rule_base.root_name, 'Table_Consistency', context,
                                                      (rule.pattern(14),
                                                       rule.pattern(15),
                                                       rule.pattern(16),)) \
                                      as gen_8:
                                      for x_8 in gen_8:
                                        assert x_8 is None, \
                                          "ness_check.Consistent_data: got unexpected plan from when clause 8"
                                        rule.rule_base.num_bc_rule_successes += 1
                                        yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def check_range(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        r = int(context.lookup_data('nnode')[0])
        if int(context.lookup_data('index')[0]) in range(r):
          rule.rule_base.num_bc_rule_successes += 1
          yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def Table_Consistency(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'Check_Servers', context,
                          (rule.pattern(0),
                           rule.pattern(1),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "ness_check.Table_Consistency: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'Flags_Consistency', context,
                              (rule.pattern(1),
                               rule.pattern(2),)) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "ness_check.Table_Consistency: got unexpected plan from when clause 2"
                rule.rule_base.num_bc_rule_successes += 1
                yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def Check_Servers(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        for python_ans in \
             context.lookup_data('servers_list'):
          mark1 = context.mark(True)
          if rule.pattern(0).match_data(context, context, python_ans):
            context.end_save_all_undo()
            if context.lookup_data('serv') in context.lookup_data('good_server_status_list'):
              rule.rule_base.num_bc_rule_successes += 1
              yield
          else: context.end_save_all_undo()
          context.undo_to_mark(mark1)
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def Flags_Consistency(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        if len(context.lookup_data('servers_list')) == len(context.lookup_data('flags_list')):
          rule.rule_base.num_bc_rule_successes += 1
          yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def Status_Analysis_g(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'Check_flags', context,
                          (rule.pattern(0),
                           rule.pattern(1),
                           rule.pattern(2),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "ness_check.Status_Analysis_g: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'Valid_Status', context,
                              (rule.pattern(0),
                               rule.pattern(2),)) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "ness_check.Status_Analysis_g: got unexpected plan from when clause 2"
                rule.rule_base.num_bc_rule_successes += 1
                yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def Status_Analysis_u(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        with engine.prove(rule.rule_base.root_name, 'Check_flags_u', context,
                          (rule.pattern(0),
                           rule.pattern(1),
                           rule.pattern(2),)) \
          as gen_1:
          for x_1 in gen_1:
            assert x_1 is None, \
              "ness_check.Status_Analysis_u: got unexpected plan from when clause 1"
            with engine.prove(rule.rule_base.root_name, 'Uncertain_Status', context,
                              (rule.pattern(0),
                               rule.pattern(2),)) \
              as gen_2:
              for x_2 in gen_2:
                assert x_2 is None, \
                  "ness_check.Status_Analysis_u: got unexpected plan from when clause 2"
                rule.rule_base.num_bc_rule_successes += 1
                yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def Valid_Status(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        if int(context.lookup_data('latest_status_list')[int(context.lookup_data('index')[0])]) == 1:
          rule.rule_base.num_bc_rule_successes += 1
          yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def Uncertain_Status(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        if int(context.lookup_data('latest_status_list')[int(context.lookup_data('index')[0])]) == 3:
          rule.rule_base.num_bc_rule_successes += 1
          yield
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def Check_flags(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        ref = context.lookup_data('latest_status_list')[int(context.lookup_data('index')[0])]
        count = 0
        for i in context.lookup_data('flags_list'):
                if i == ref:
                        count = count + 1
        if count >= len(context.lookup_data('flags_list'))/2:
                res = 1
        else:
                res = 0
        mark2 = context.mark(True)
        if rule.pattern(0).match_data(context, context,
                res):
          context.end_save_all_undo()
          if context.lookup_data('ans') == 1:
            rule.rule_base.num_bc_rule_successes += 1
            yield
        else: context.end_save_all_undo()
        context.undo_to_mark(mark2)
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def Check_flags_u(rule, arg_patterns, arg_context):
  engine = rule.rule_base.engine
  patterns = rule.goal_arg_patterns()
  if len(arg_patterns) == len(patterns):
    context = contexts.bc_context(rule)
    try:
      if all(map(lambda pat, arg:
                   pat.match_pattern(context, context,
                                     arg, arg_context),
                 patterns,
                 arg_patterns)):
        rule.rule_base.num_bc_rules_matched += 1
        ref = context.lookup_data('latest_status_list')[int(context.lookup_data('index')[0])]
        count_1 = 0
        count_2 = 0
        for i in context.lookup_data('flags_list'):
                if i == 1:
                        count_1 = count_1 + 1
                if i == 2:
                        count_2 = count_2 + 1
        if count_1 > len(context.lookup_data('flags_list'))/2:
                res = 1
        else:
                if count_2 > len(context.lookup_data('flags_list'))/2:
                        res = 2
                else:
                        res = 3
        mark2 = context.mark(True)
        if rule.pattern(0).match_data(context, context,
                res):
          context.end_save_all_undo()
          if context.lookup_data('ans') == 3:
            rule.rule_base.num_bc_rule_successes += 1
            yield
        else: context.end_save_all_undo()
        context.undo_to_mark(mark2)
        rule.rule_base.num_bc_rule_failures += 1
    finally:
      context.done()

def populate(engine):
  This_rule_base = engine.get_create('ness_check')
  
  bc_rule.bc_rule('trust_analysis', This_rule_base, 'trust_analysis',
                  trust_analysis, None,
                  (pattern.pattern_literal('Good'),),
                  (),
                  (pattern.pattern_literal('latest_status_list'),
                   pattern.pattern_tuple((), contexts.variable('latest_status_list')),
                   pattern.pattern_literal('good_server_status_list'),
                   pattern.pattern_tuple((), contexts.variable('good_server_status_list')),
                   pattern.pattern_literal('flags_list'),
                   pattern.pattern_tuple((), contexts.variable('flags_list')),
                   pattern.pattern_literal('servers_list'),
                   pattern.pattern_tuple((), contexts.variable('servers_list')),
                   pattern.pattern_literal('index'),
                   pattern.pattern_tuple((), contexts.variable('index')),
                   pattern.pattern_literal('number_nodes'),
                   pattern.pattern_tuple((), contexts.variable('nnode')),
                   contexts.variable('nnode'),
                   contexts.variable('index'),
                   contexts.variable('good_server_status_list'),
                   contexts.variable('servers_list'),
                   contexts.variable('flags_list'),
                   contexts.variable('latest_status_list'),))
  
  bc_rule.bc_rule('trust_analysis_1', This_rule_base, 'trust_analysis',
                  trust_analysis_1, None,
                  (pattern.pattern_literal('Uncertain'),),
                  (),
                  (pattern.pattern_literal('latest_status_list'),
                   pattern.pattern_tuple((), contexts.variable('latest_status_list')),
                   pattern.pattern_literal('good_server_status_list'),
                   pattern.pattern_tuple((), contexts.variable('good_server_status_list')),
                   pattern.pattern_literal('flags_list'),
                   pattern.pattern_tuple((), contexts.variable('flags_list')),
                   pattern.pattern_literal('servers_list'),
                   pattern.pattern_tuple((), contexts.variable('servers_list')),
                   pattern.pattern_literal('index'),
                   pattern.pattern_tuple((), contexts.variable('index')),
                   pattern.pattern_literal('number_nodes'),
                   pattern.pattern_tuple((), contexts.variable('nnode')),
                   contexts.variable('nnode'),
                   contexts.variable('index'),
                   contexts.variable('good_server_status_list'),
                   contexts.variable('servers_list'),
                   contexts.variable('flags_list'),
                   contexts.variable('latest_status_list'),))
  
  bc_rule.bc_rule('Consistent_data', This_rule_base, 'consistency_analysis',
                  Consistent_data, None,
                  (pattern.pattern_literal('Good'),),
                  (),
                  (pattern.pattern_literal('latest_status_list'),
                   pattern.pattern_tuple((), contexts.variable('latest_status_list')),
                   pattern.pattern_literal('good_server_status_list'),
                   pattern.pattern_tuple((), contexts.variable('good_server_status_list')),
                   pattern.pattern_literal('flags_list'),
                   pattern.pattern_tuple((), contexts.variable('flags_list')),
                   pattern.pattern_literal('servers_list'),
                   pattern.pattern_tuple((), contexts.variable('servers_list')),
                   pattern.pattern_literal('index'),
                   pattern.pattern_tuple((), contexts.variable('index')),
                   pattern.pattern_literal('number_nodes'),
                   pattern.pattern_tuple((), contexts.variable('nnode')),
                   contexts.variable('nnode'),
                   contexts.variable('index'),
                   contexts.variable('good_server_status_list'),
                   contexts.variable('servers_list'),
                   contexts.variable('flags_list'),))
  
  bc_rule.bc_rule('check_range', This_rule_base, 'check_range',
                  check_range, None,
                  (contexts.variable('nnode'),
                   contexts.variable('index'),),
                  (),
                  ())
  
  bc_rule.bc_rule('Table_Consistency', This_rule_base, 'Table_Consistency',
                  Table_Consistency, None,
                  (contexts.variable('good_server_status_list'),
                   contexts.variable('servers_list'),
                   contexts.variable('flags_list'),),
                  (),
                  (contexts.variable('good_server_status_list'),
                   contexts.variable('servers_list'),
                   contexts.variable('flags_list'),))
  
  bc_rule.bc_rule('Check_Servers', This_rule_base, 'Check_Servers',
                  Check_Servers, None,
                  (contexts.variable('good_server_status_list'),
                   contexts.variable('servers_list'),),
                  (),
                  (contexts.variable('serv'),))
  
  bc_rule.bc_rule('Flags_Consistency', This_rule_base, 'Flags_Consistency',
                  Flags_Consistency, None,
                  (contexts.variable('servers_list'),
                   contexts.variable('flags_list'),),
                  (),
                  ())
  
  bc_rule.bc_rule('Status_Analysis_g', This_rule_base, 'Status_Analysis',
                  Status_Analysis_g, None,
                  (contexts.variable('latest_status_list'),
                   contexts.variable('flags_list'),
                   contexts.variable('index'),),
                  (),
                  (contexts.variable('latest_status_list'),
                   contexts.variable('flags_list'),
                   contexts.variable('index'),))
  
  bc_rule.bc_rule('Status_Analysis_u', This_rule_base, 'Status_Analysis_u',
                  Status_Analysis_u, None,
                  (contexts.variable('latest_status_list'),
                   contexts.variable('flags_list'),
                   contexts.variable('index'),),
                  (),
                  (contexts.variable('latest_status_list'),
                   contexts.variable('flags_list'),
                   contexts.variable('index'),))
  
  bc_rule.bc_rule('Valid_Status', This_rule_base, 'Valid_Status',
                  Valid_Status, None,
                  (contexts.variable('latest_status_list'),
                   contexts.variable('index'),),
                  (),
                  ())
  
  bc_rule.bc_rule('Uncertain_Status', This_rule_base, 'Uncertain_Status',
                  Uncertain_Status, None,
                  (contexts.variable('latest_status_list'),
                   contexts.variable('index'),),
                  (),
                  ())
  
  bc_rule.bc_rule('Check_flags', This_rule_base, 'Check_flags',
                  Check_flags, None,
                  (contexts.variable('latest_status_list'),
                   contexts.variable('flags_list'),
                   contexts.variable('index'),),
                  (),
                  (contexts.variable('ans'),))
  
  bc_rule.bc_rule('Check_flags_u', This_rule_base, 'Check_flags_u',
                  Check_flags_u, None,
                  (contexts.variable('latest_status_list'),
                   contexts.variable('flags_list'),
                   contexts.variable('index'),),
                  (),
                  (contexts.variable('ans'),))


Krb_filename = '../ness_check.krb'
Krb_lineno_map = (
    ((14, 18), (12, 12)),
    ((20, 26), (14, 14)),
    ((27, 33), (15, 15)),
    ((34, 40), (16, 16)),
    ((41, 47), (17, 17)),
    ((48, 54), (18, 18)),
    ((55, 61), (19, 19)),
    ((62, 67), (20, 26)),
    ((68, 74), (27, 27)),
    ((75, 82), (28, 28)),
    ((83, 90), (29, 29)),
    ((103, 107), (32, 32)),
    ((109, 115), (34, 34)),
    ((116, 122), (35, 35)),
    ((123, 129), (36, 36)),
    ((130, 136), (37, 37)),
    ((137, 143), (38, 38)),
    ((144, 150), (39, 39)),
    ((151, 157), (40, 40)),
    ((158, 165), (41, 41)),
    ((166, 173), (42, 42)),
    ((186, 190), (45, 45)),
    ((192, 198), (47, 47)),
    ((199, 205), (48, 48)),
    ((206, 212), (49, 49)),
    ((213, 219), (50, 50)),
    ((220, 226), (51, 51)),
    ((227, 233), (52, 52)),
    ((234, 240), (53, 53)),
    ((241, 248), (54, 54)),
    ((261, 265), (57, 57)),
    ((267, 267), (59, 59)),
    ((268, 268), (60, 60)),
    ((281, 285), (63, 63)),
    ((287, 293), (65, 65)),
    ((294, 300), (66, 66)),
    ((313, 317), (69, 69)),
    ((320, 320), (71, 71)),
    ((324, 324), (72, 72)),
    ((339, 343), (75, 75)),
    ((345, 345), (77, 77)),
    ((358, 362), (80, 80)),
    ((364, 371), (82, 82)),
    ((372, 378), (83, 83)),
    ((391, 395), (86, 86)),
    ((397, 404), (88, 88)),
    ((405, 411), (89, 89)),
    ((424, 428), (92, 92)),
    ((430, 430), (94, 94)),
    ((443, 447), (97, 97)),
    ((449, 449), (99, 99)),
    ((462, 466), (102, 102)),
    ((468, 476), (104, 113)),
    ((479, 479), (114, 114)),
    ((481, 481), (115, 115)),
    ((496, 500), (118, 118)),
    ((502, 516), (120, 135)),
    ((519, 519), (136, 136)),
    ((521, 521), (137, 137)),
)
