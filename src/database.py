import sqlite3
import rospy

# wrapper for SQLlite3 database
class TopicModel:
    # initialize database if it hasnt been already
    def __init__(self):
        rospy.logdebug("Connecting to database")
        self._db = sqlite3.connect(":memory:", check_same_thread=False)
        self._db.row_factory = lambda cursor, row: row
        try:
            self._db.cursor().execute(
                """
                    DROP TABLE topics
                    """
            )
        except Exception:
            pass

        # add table to database for each topic
        # title: plaintext description of topic
        # handle: topic address in ROS
        # rate: rate in hz as a string
        # target: target rate in hz as a string
        # type: ROS message type
        # message: message contents as a string
        # publishers: list of publishers for a certain topic
        # subscribers: list of subscribers to a certain topic
        self._db.cursor().execute(
            """
            CREATE TABLE topics(
                id INTEGER PRIMARY KEY,
                title TEXT,
                handle TEXT,
                rate TEXT,
                target TEXT,
                type TEXT,
                message TEXT,
                publishers TEXT,
                subscribers TEXT)
        """
        )

        # commit changes
        self._db.commit()

        # set variable for saving currently selected topic in display
        self.current_id = None

    # add topic into database for list view
    def add(self, topic):
        self._db.cursor().execute(
            """
            INSERT INTO topics(title, handle, rate, target, type)
            VALUES(:title, :handle, :rate, :target, :type)""",
            topic,
        )
        self._db.commit()

    # get all topics in database for list view
    def get_summary(self):
        rows = self._db.cursor().execute(
            "SELECT title, handle, rate, target, type, id FROM topics"
        )
        return rows.fetchall()

    # get topic by id
    def get_topic(self, topic_id):
        return (
            self._db.cursor()
            .execute("SELECT * from topics where id=:id", (topic_id,))
            .fetchone()
        )

    # get currently selected topic, if there is one
    def get_current_topic(self):
        if self.current_id is None:
            return (0, "", "", "0.0", "0.0", "", "", "", "")
        else:
            return self.get_topic(self.current_id)

    # update current topic in all fields
    def update_current_topic(self, data):
        if self.current_id is None:
            self.add(data)
        else:
            self._db.cursor().execute(
                """
                UPDATE topics SET title=:title, handle=:handle, rate=:rate, 
                target=:target, type=:type WHERE id=:id""",
                data,
            )
            self._db.commit()

    # update topic hz by id
    def update_topic_hz(self, hz, topic_id):
        self._db.cursor().execute(
            """
            UPDATE topics SET rate=:rate WHERE id=:id
            """,
            (hz, topic_id),
        )

    # update topic info in topic focus view
    def update_topic_info(self, message, publishers, subscribers):
        self._db.cursor().execute(
            """
            UPDATE topics SET message=:message, publishers=:publishers, 
            subscribers=:subscribers WHERE id=:id
            """,
            (message, publishers, subscribers, self.current_id),
        )

    # delete a topic from database by id
    def delete_topic(self, topic_id):
        self._db.cursor().execute(
            """
            DELETE FROM topics WHERE id=:id""",
            {"id": topic_id},
        )
        self._db.commit()
