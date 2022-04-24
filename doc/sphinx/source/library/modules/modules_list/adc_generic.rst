.. library modules_list adc_generic

======================
ADC generic
======================

If you want to receive the value of some ADC channel, you can use the "ADC Generic" module. When activated, the aircraft sends 2 values corresponding to the selected ADC channels. They can be read from the "Messages" application. Add the adc_generic to your modules:

.. code-block:: xml

  <modules>
    ...
    <load name="adc_generic.xml">
      <configure name="ADC_CHANNEL_GENERIC1" value="ADC_3"/>
      <configure name="ADC_CHANNEL_GENERIC2" value="ADC_4"/>
    </load>
  </modules>

In this example, the ADC channels 3 and 4 are read and sent by telemetry at 4Hz:

.. code-block:: xml

  <message name="ADC_GENERIC" ID="18">
    <field name="val1" type="uint16"/>
    <field name="val2" type="uint16"/>
  </message>

Only one or two channels can be defined. If only one is activated, 0 will be sent for the unused value.