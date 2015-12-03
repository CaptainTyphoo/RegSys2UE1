
function killing_babycats(block)

setup(block);


%%
function setup(block)

  % Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 2;
  
  % Register number of continuous states
  block.NumContStates = 2;
  
  % Register dialog parameter
  block.NumDialogPrms = 1; 
  
  % Port dimensions
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).SamplingMode = 'Sample';
  block.InputPort(1).DirectFeedthrough = false;
  
  block.OutputPort(1).Dimensions       = 2;
  block.OutputPort(1).SamplingMode = 'Sample';  
  block.OutputPort(2).Dimensions       = 1;
  block.OutputPort(2).SamplingMode = 'Sample';  
  
  % Set block sample time to continuous time
  block.SampleTimes = [0 0];
  
  % Register methods
  block.RegBlockMethod('InitializeConditions',    @InitializeConditions); 
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Derivatives',             @Derivatives);
  block.RegBlockMethod('Terminate',               @Terminate);

%end setup


function InitializeConditions(block)
 x0 = block.DialogPrm(1).Data;
block.ContStates.Data=x0;

function Output(block)

 x = block.ContStates.Data;
 u = block.InputPort(1).Data;
  
block.OutputPort(1).Data = x;
block.OutputPort(2).Data = u;
  

function Derivatives(block)
  
  x = block.ContStates.Data;
  u1 = block.InputPort(1).Data;
  
  dx(1)=-x(1)+x(1)*x(2);
  dx(2)= x(2)-x(1)*x(2)+x(2)*u1;
  
  block.Derivatives.Data=dx;  

function Terminate(block)

%end Terminate

