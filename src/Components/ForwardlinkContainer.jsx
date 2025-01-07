import React from 'react';
import {
  Container,
  Rows
} from './ContainerStyles';

function ForwardlinkContainer() {
  return (
    <Container>
      <Rows>
        <div>Forward</div>
      </Rows>
    </Container>
  )
}

export default React.memo(ForwardlinkContainer)