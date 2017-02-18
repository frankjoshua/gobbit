Blockly.Blocks['move'] = {
  init: function() {
    this.appendValueInput("twist value")
        .setCheck(null)
        .appendField(new Blockly.FieldNumber(0), "twist value");
    this.setPreviousStatement(true, null);
    this.setNextStatement(true, null);
    this.setColour(230);
    this.setTooltip('this publishes a twist message');
    this.setHelpUrl('');
  }
};