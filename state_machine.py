
class StateMachine(object):

    class Transition(object):

        def __init__(self):

            # This is a list of boolean functions that tested. If
            # all functions return true then we can go to the next state
            self.conditions = list()

            self.next_state = None

        # A condition is just a function that must evaluate to true or false
        def add_condition(self, function_condition):
            self.conditions.append(function_condition)

        # check to see if the conditions of the transmission all evaluate to true
        def all_conditions_met(self):

            # condition list is empty
            if not self.conditions:
                return False

            for condition in self.conditions:

                # condition failed
                if not condition():
                    return False
            # all conditions evaluated to true
            return True

    class State(object):
        def __init__(self, name="default"):
            self.name = name

            # a list of transitions to some other state
            self.transitions = list()

        #.When there is a transition to a new state
        # this function automatically called.
        def restart(self):
            pass

        def add_transition(self, new_transition):
            self.transitions.append(new_transition)

        def __eq__(self, other):
            return self.name == other.name

    def __init__(self):
        self.states = list()
        self.current_state = None

    # this function is to update the state machine based on the conditions of the current state
    def update(self):

        # this is the transition that we can traverse between states if all of its conditions are met
        valid_transition = None

        # go through every transition of the current state
        for transition in self.current_state.transitions:

            # make sure that all conditions are met for this transition
            if transition.all_conditions_met():
                valid_transition = transition
                break

        # go to the next state
        if valid_transition is not None:
            self.current_state = valid_transition.next_state
            self.current_state.restart()
            self.state_changed()

    def add_state(self, new_state):
        self.states.append(new_state)

    # Creates a transition from a to b and from b to a
    def add_bi_transition(self, state_a_name, state_b_name, transition_ab, transition_ba):

        state_a = self.get_state(state_a_name)
        state_b = self.get_state(state_b_name)

        # failed to find the states
        if state_a is None or state_b is None:
            print("Error, invalid state specified")
            return

        transition_ab.next_state = state_b
        transition_ba.next_state = state_a

        state_a.add_transition(transition_ab)
        state_b.add_transition(transition_ba)

    def add_transition_from(self, state_a_name, state_b_name, new_transition):

        state_a = self.get_state(state_a_name)
        state_b = self.get_state(state_b_name)

        # failed to find the states
        if state_a is None or state_b is None:
            print("Error, invalid state specified")
            return

        # add a transition from a to b
        new_transition.next_state = state_b
        state_a.add_transition(new_transition)

    def get_state(self, state_name):
        for state in self.states:
            if state.name == state_name:
                return state
        return None

    def set_current_state(self, state_name):
        for state in self.states:
            if state.name == state_name:
                self.current_state = state
                break

    # Gets called when there was a change of states.
    # When this is called "current_state" will already have been updated
    # to the new state.
    def state_changed(self):
        pass


# A special type of state machine for switching between animations
class AnimationStateMachine(StateMachine):

    class AnimationState(StateMachine.State):
        def __init__(self, name, animation):
            super(AnimationStateMachine.AnimationState, self).__init__(name)
            self.animation = animation

    def __init__(self, animator):
        super(AnimationStateMachine, self).__init__()

        # the animator associated for this animation state machine
        self.animator = animator

    # update the animator if there was a change of states
    def state_changed(self):
        # set the animation from the new state
        self.animator.set_animation(self.current_state.animation)

    # update the animator's current animation and the current state
    def set_current_state(self, state_name):
        for state in self.states:
            if state.name == state_name:
                self.current_state = state
                self.animator.current_animation = self.current_state.animation
                break
