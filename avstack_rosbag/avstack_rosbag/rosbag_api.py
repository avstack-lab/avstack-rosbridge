import sqlite3


def get_database_names(cursor):
    cursor.execute("PRAGMA database_list")
    rows = cursor.fetchall()
    dbs = [row[2] for row in rows]
    return dbs


def connect(sqlite_file):
    """Make connection to an SQLite database file."""
    try:
        conn = sqlite3.connect(sqlite_file)
    except Exception:
        raise RuntimeError(f"Could not connect to {sqlite_file} database")
    c = conn.cursor()
    return conn, c


def close(conn):
    """Close connection to the database."""
    conn.close()


def countRows(cursor, table_name, print_out=False):
    """Returns the total number of rows in the database."""
    cursor.execute("SELECT COUNT(*) FROM {}".format(table_name))
    count = cursor.fetchall()
    if print_out:
        print("\nTotal rows: {}".format(count[0][0]))
    return count[0][0]


def getHeaders(cursor, table_name, print_out=False):
    """Returns a list of tuples with column informations:
    (id, name, type, notnull, default_value, primary_key)
    """
    # Get headers from table "table_name"
    cursor.execute("PRAGMA TABLE_INFO({})".format(table_name))
    info = cursor.fetchall()
    if print_out:
        print("\nColumn Info:\nID, Name, Type, NotNull, DefaultVal, PrimaryKey")
        for col in info:
            print(col)
    return info


def getAllElements(cursor, table_name, print_out=False):
    """Returns a dictionary with all elements of the table database."""
    # Get elements from table "table_name"
    cursor.execute("SELECT * from({})".format(table_name))
    records = cursor.fetchall()
    if print_out:
        print("\nAll elements:")
        for row in records:
            print(row)
    return records


def isTopic(cursor, topic_name, print_out=False):
    """Returns topic_name header if it exists. If it doesn't, returns empty.
    It returns the last topic found with this name.
    """
    boolIsTopic = False
    topicFound = []

    # Get all records for 'topics'
    records = getAllElements(cursor, "topics", print_out=False)

    # Look for specific 'topic_name' in 'records'
    for row in records:
        if row[1] == topic_name:  # 1 is 'name' TODO
            boolIsTopic = True
            topicFound = row
    if print_out:
        if boolIsTopic:
            # 1 is 'name', 0 is 'id' TODO
            print("\nTopic named", topicFound[1], " exists at id ", topicFound[0], "\n")
        else:
            print("\nTopic", topic_name, "could not be found. \n")

    return topicFound


def getAllMessagesInTopic(cursor, topic_name, print_out=False):
    """Returns all timestamps and messages at that topic.
    There is no deserialization for the BLOB data.
    """
    count = 0
    timestamps = []
    messages = []

    # Find if topic exists and its id
    topicFound = isTopic(cursor, topic_name, print_out=False)

    # If not find return empty
    if not topicFound:
        print("Topic", topic_name, "could not be found. \n")
        breakpoint()

    else:
        records = getAllElements(cursor, "messages", print_out=False)

        # Look for message with the same id from the topic
        for row in records:
            if row[1] == topicFound[0]:  # 1 and 0 is 'topic_id' TODO
                count = count + 1  # count messages for this topic
                timestamps.append(row[2])  # 2 is for timestamp TODO
                messages.append(row[3])  # 3 is for all messages

        # Print
        if print_out:
            print("\nThere are ", count, "messages in ", topicFound[1])

    return timestamps, messages


def getAllTopicsNames(cursor, print_out=False):
    """Returns all topics names."""
    topicNames = []
    # Get all records for 'topics'
    records = getAllElements(cursor, "topics", print_out=False)

    # Save all topics names
    for row in records:
        topicNames.append(row[1])  # 1 is for topic name TODO
    if print_out:
        print("\nTopics names are:")
        print(topicNames)

    return topicNames


def getAllMsgsTypes(cursor, print_out=False):
    """Returns all messages types."""
    msgsTypes = []
    # Get all records for 'topics'
    records = getAllElements(cursor, "topics", print_out=False)

    # Save all message types
    for row in records:
        msgsTypes.append(row[2])  # 2 is for message type TODO
    if print_out:
        print("\nMessages types are:")
        print(msgsTypes)

    return msgsTypes


def getMsgType(cursor, topic_name, print_out=False):
    """Returns the message type of that specific topic."""
    msg_type = []
    # Get all topics names and all message types
    topic_names = getAllTopicsNames(cursor, print_out=False)
    msgs_types = getAllMsgsTypes(cursor, print_out=False)

    # look for topic at the topic_names list, and find its index
    for index, element in enumerate(topic_names):
        if element == topic_name:
            msg_type = msgs_types[index]
    if print_out:
        print("\nMessage type in", topic_name, "is", msg_type)

    return msg_type
