import header
from main import *
from common import utils
import traceback

"""
mut = mutual.Mutual(MUTUALINT)
mut.test()
ca = ca_main.CA(random.randint(1000, 64000))

# Continuous authentication single node
ca.test()

# Continuous authentication multiple nodes
#prerequisite: establish mesh
if not mesh_utils.verify_mesh_status():  # verifying that mesh is running
    print("Mesh network not established")
    only_mesh()
ca.test_multiple_nodes(num_nodes)

# Table exchange single node
ca.test_exchange_table()

# Table exchange multiple nodes
#prerequisite: establish mesh and run one round of CA
if not mesh_utils.verify_mesh_status():  # verifying that mesh is running
    print("Mesh network not established")
    only_mesh()
myID = pri.get_labels()
only_ca(myID)
ca.test_exchange_table_multiple_nodes()

# Decision engine
ness = ness_main.NESS()
ness.test()

# MBA single node
m = mba.MBA('127.0.0.1')
m.test()

# Quarantine single node
qua = quarantine.Quarantine()
qua.test()
"""
def sec_beat_test(num_nodes):
    """
    prerequisite: mesh must be established between the nodes
    unit test should be run as
    from test import *
    sec_beat_test(num_nodes)
    stores log to logs/sec_best_{num_nodes}_nodes-log.txt
    """
    common_ut = utils.Utils()
    logger = common_ut.setup_logger(f'sec_beat_{num_nodes}_nodes')

    mut = mutual.Mutual(MUTUALINT)
    myID = mut.myID

    #sec_beat(myID)
    q = queue.Queue()
    ut.checkiptables()
    ma = mba.MBA(mesh_utils.get_mesh_ip_address())
    # Start exchange table server so that it can receive messages as soon as other nodes complete cont auth
    start_server_thread = ut.start_server()
    # Continuous authentication
    try:
        sectable = only_ca(myID)
        sectable.drop_duplicates(inplace=True)
        logger.info("Continuous authentication executed")
        logger.debug("Security table after continuous authentication:\n%s", sectable)
    except Exception as e:
        print("Continuous authentication failed with exception ", e)
        traceback.print_exc()
        logger.error("Continuous authentication failed with exception %s", e)
        common_ut.close_logger(logger)
        return
    # Exchange table
    try:
        ut.exchage_table(sectable, start_server_thread)
        global_table = pd.read_csv('auth/global_table.csv')
        logger.info("Exchange table executed")
        logger.debug("Global table after exchange:\n%s", global_table)
    except Exception as e:
        print("Exchange table failed with exception ", e)
        traceback.print_exc()
        logger.error("Exchange table failed with exception %s", e)
        common_ut.close_logger(logger)
        return
    # Decision engine
    try:
        ness_result, mapp = decision_engine(global_table, ma, q)
        logger.info("Decision engine executed")
        logger.debug("Ness result:\n%s", ness_result)
    except Exception as e:
        print("Decision engine failed with exception ", e)
        traceback.print_exc()
        logger.error("Decision engine failed with exception %s", e)
        common_ut.close_logger(logger)
        return
    # Quarantine
    try:
        quaran(ness_result, q, global_table, ma, mapp)
        logger.info("MBA and quarantine executed")
    except Exception as e:
        print("Quarantine failed with exception ", e)
        traceback.print_exc()
        logger.error("Quarantine failed with exception %s", e)

    common_ut.close_logger(logger)

#sec_beat_test(2)