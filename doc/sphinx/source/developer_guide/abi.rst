.. developer_guide abi

=============
ABI messaging
=============

Paparazzi use a custom publish/subscribe middleware to exchange data between software components named ``ABI``.

.. note:: This middleware was named *ABI* for *AirBorne Ivy*

General Idea
------------

* Give an easy way to allow software components to exchange data with minimum delay nor execution overhead.
* The components don't need to know each other, they only need the message format.
* Each subscriber set a callback function that will be called when new data are sent.


Message definition
------------------

The messages are described in ``conf/abi.xml`` analoguous to other messages in paparazzi. Name are unique, IDs starts from 0. "type" field should be a real C type. Field name might be useless.

.. code-block:: xml

    <class name="airborne">
     <message name="DATA" id="0">
      <field name="a" type="float"/>
      <field name="b" type="struct bla"/>
     </message>
     ...
    </class>

Airborne code for subscriber
----------------------------

Include header and declare an ``abi_event`` as a global variable. Write the callback function with the proper prototype, matching the message.

.. code-block:: C

    #include "subsystems/abi.h"

    abi_event ev;

    void data_cb(uint8_t sender_id, const float * a, const struct bla * b) {
     // do something here
    }

In the initialization function (or later) call the binding function for the message you want to receive.

.. code-block:: C

    AbiBindMsgDATA(ABI_BROADCAST, &ev, data_cb);


The first parameter is the sender ID you want to receive the message from.

* ``ABI_BROADCAST`` is used to receive messages from all senders.
* ``ABI_DISABLE`` disable the callback (it will never be called).
* Senders IDs can be found in the file ``sw/airborne/subsystems/abi_sender_ids.h``

The second parameter is a pointer to the global ``abi_event`` you declared. This variable **can't** be reused for another bind. You must declare one abi_event per bind.

The last parameter is your callback.


Airborne code for publisher
---------------------------

Include header, then call the send function with the appropriate parameters

.. code-block:: C

    float var = 2.;
    struct bla s;
    AbiSendMsgDATA(SENDER_ID, &var, &s);

Replace ``SENDER_ID`` by your sender ID defined in ``sw/airborne/subsystems/abi_sender_ids.h``.

Your sender ID identifier should be constructed as the concatenation of the name of the message and the name of your module, suffixed with ``_ID``.

.. admonition:: example

    A good sender ID for a module ``toto`` sending the message ``DATA`` may be ``DATA_TOTO_ID``

.. warning::

    The values 0 and 255 are reserved for ``ABI_DISABLE`` and ``ABI_BROADCAST`` and thus shall not be used.
    
    You must also avoid using an ID already used to send the same message type.


Code generation
---------------

The generated code will be in ``var/include/abi_messages.h`` and include some structure definition from ``sw/airborne/subsystems/abi_common.h`` (``sw/airborne/subsystems/abi.h`` is a convenience header that only includes ``var/include/abi_messages.h``). Bind and Send functions are generated, as well as callback type definition. A linked list is used to store the binded callbacks for each message. The head of the list is in an array to allow a fast access. 



